/***************************************************************************
 *   Copyright (C) 2009 by Duane Ellis                                     *
 *   openocd@duaneellis.com                                                *
 *                                                                         *
 *   Copyright (C) 2010 by Olaf Lüke (at91sam3s* support)                  *
 *   olaf@uni-paderborn.de                                                 *
 *																		   *
 *   Copyright (C) 2011 by Olivier Schonken (at91sam3x* support)           *                                          *
 *                     and Jim Norris                                      *
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
****************************************************************************/

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

/* at91sam3u series (has one or two flash banks) */
#define FLASH_BANK0_BASE_U   0x00080000
#define FLASH_BANK1_BASE_U   0x00100000

/* at91sam3s series (has always one flash bank) */
#define FLASH_BANK_BASE_S   0x00400000

/* at91sam3sd series (has always two flash banks) */
#define FLASH_BANK0_BASE_SD FLASH_BANK_BASE_S
#define FLASH_BANK1_BASE_512K_SD (FLASH_BANK0_BASE_SD+(512*1024/2))


/* at91sam3n series (has always one flash bank) */
#define FLASH_BANK_BASE_N   0x00400000

/* at91sam3a/x series has two flash banks*/
#define	FLASH_BANK0_BASE_AX			0x00080000
/*Bank 1 of the at91sam3a/x series starts at 0x00080000 + half flash size*/
#define	FLASH_BANK1_BASE_256K_AX	0x000A0000
#define	FLASH_BANK1_BASE_512K_AX	0x000C0000

#define         AT91C_EFC_FCMD_GETD                 (0x0)	/* (EFC) Get Flash Descriptor */
#define         AT91C_EFC_FCMD_WP                   (0x1)	/* (EFC) Write Page */
#define         AT91C_EFC_FCMD_WPL                  (0x2)	/* (EFC) Write Page and Lock */
#define         AT91C_EFC_FCMD_EWP                  (0x3)	/* (EFC) Erase Page and Write Page */
#define         AT91C_EFC_FCMD_EWPL                 (0x4)	/* (EFC) Erase Page and Write Page then Lock */
#define         AT91C_EFC_FCMD_EA                   (0x5)	/* (EFC) Erase All */
/* cmd6 is not present in the at91sam3u4/2/1 data sheet table 17-2 */
/* #define      AT91C_EFC_FCMD_EPL                  (0x6) // (EFC) Erase plane? */
/* cmd7 is not present in the at91sam3u4/2/1 data sheet table 17-2 */
/* #define      AT91C_EFC_FCMD_EPA                  (0x7) // (EFC) Erase pages? */
#define         AT91C_EFC_FCMD_SLB                  (0x8)	/* (EFC) Set Lock Bit */
#define         AT91C_EFC_FCMD_CLB                  (0x9)	/* (EFC) Clear Lock Bit */
#define         AT91C_EFC_FCMD_GLB                  (0xA)	/* (EFC) Get Lock Bit */
#define         AT91C_EFC_FCMD_SFB                  (0xB)	/* (EFC) Set Fuse Bit */
#define         AT91C_EFC_FCMD_CFB                  (0xC)	/* (EFC) Clear Fuse Bit */
#define         AT91C_EFC_FCMD_GFB                  (0xD)	/* (EFC) Get Fuse Bit */
#define         AT91C_EFC_FCMD_STUI                 (0xE)	/* (EFC) Start Read Unique ID */
#define         AT91C_EFC_FCMD_SPUI                 (0xF)	/* (EFC) Stop Read Unique ID */

#define  OFFSET_EFC_FMR   0
#define  OFFSET_EFC_FCR   4
#define  OFFSET_EFC_FSR   8
#define  OFFSET_EFC_FRR   12

extern const struct flash_driver at91sam3_flash;

static float _tomhz(uint32_t freq_hz)
{
	float f;

	f = ((float)(freq_hz)) / 1000000.0;
	return f;
}

/* How the chip is configured. */
struct sam3_cfg {
	uint32_t unique_id[4];

	uint32_t slow_freq;
	uint32_t rc_freq;
	uint32_t mainosc_freq;
	uint32_t plla_freq;
	uint32_t mclk_freq;
	uint32_t cpu_freq;
	uint32_t fclk_freq;
	uint32_t pclk0_freq;
	uint32_t pclk1_freq;
	uint32_t pclk2_freq;


#define SAM3_CHIPID_CIDR          (0x400E0740)
	uint32_t CHIPID_CIDR;
#define SAM3_CHIPID_CIDR2         (0x400E0940) /*SAM3X and SAM3A cidr at this address*/
	uint32_t CHIPID_CIDR2;
#define SAM3_CHIPID_EXID          (0x400E0744)
	uint32_t CHIPID_EXID;
#define SAM3_CHIPID_EXID2         (0x400E0944) /*SAM3X and SAM3A cidr at this address*/
	uint32_t CHIPID_EXID2;


#define SAM3_PMC_BASE             (0x400E0400)
#define SAM3_PMC_SCSR             (SAM3_PMC_BASE + 0x0008)
	uint32_t PMC_SCSR;
#define SAM3_PMC_PCSR             (SAM3_PMC_BASE + 0x0018)
	uint32_t PMC_PCSR;
#define SAM3_CKGR_UCKR            (SAM3_PMC_BASE + 0x001c)
	uint32_t CKGR_UCKR;
#define SAM3_CKGR_MOR             (SAM3_PMC_BASE + 0x0020)
	uint32_t CKGR_MOR;
#define SAM3_CKGR_MCFR            (SAM3_PMC_BASE + 0x0024)
	uint32_t CKGR_MCFR;
#define SAM3_CKGR_PLLAR           (SAM3_PMC_BASE + 0x0028)
	uint32_t CKGR_PLLAR;
#define SAM3_PMC_MCKR             (SAM3_PMC_BASE + 0x0030)
	uint32_t PMC_MCKR;
#define SAM3_PMC_PCK0             (SAM3_PMC_BASE + 0x0040)
	uint32_t PMC_PCK0;
#define SAM3_PMC_PCK1             (SAM3_PMC_BASE + 0x0044)
	uint32_t PMC_PCK1;
#define SAM3_PMC_PCK2             (SAM3_PMC_BASE + 0x0048)
	uint32_t PMC_PCK2;
#define SAM3_PMC_SR               (SAM3_PMC_BASE + 0x0068)
	uint32_t PMC_SR;
#define SAM3_PMC_IMR              (SAM3_PMC_BASE + 0x006c)
	uint32_t PMC_IMR;
#define SAM3_PMC_FSMR             (SAM3_PMC_BASE + 0x0070)
	uint32_t PMC_FSMR;
#define SAM3_PMC_FSPR             (SAM3_PMC_BASE + 0x0074)
	uint32_t PMC_FSPR;
};

/*
 * The AT91SAM3N data sheet 04-Oct-2010, AT91SAM3U data sheet 22-Aug-2011
 * and AT91SAM3S data sheet 09-Feb-2011 state that for flash writes
 * the flash wait state (FWS) should be set to 6. It seems like that the
 * cause of the problem is not the flash itself, but the flash write
 * buffer. Ie the wait states have to be set before writing into the
 * buffer.
 * Tested and confirmed with SAM3N and SAM3U
 */

struct sam3_bank_private {
	bool probed;
	/* DANGER: THERE ARE DRAGONS HERE.. */
	/* NOTE: If you add more 'ghost' pointers */
	/* be aware that you must *manually* update */
	/* these pointers in the function sam3_get_details() */
	/* See the comment "Here there be dragons" */

	/* so we can find the chip we belong to */
	struct sam3_chip *chip;
	/* so we can find the original bank pointer */
	struct flash_bank *bank;
	unsigned bank_number;
	uint32_t controller_address;
	uint32_t base_address;
	uint32_t flash_wait_states;
	bool present;
	unsigned size_bytes;
	unsigned nsectors;
	unsigned sector_size;
	unsigned page_size;
};

struct sam3_chip_details {
	/* THERE ARE DRAGONS HERE.. */
	/* note: If you add pointers here */
	/* be careful about them as they */
	/* may need to be updated inside */
	/* the function: "sam3_get_details() */
	/* which copy/overwrites the */
	/* 'runtime' copy of this structure */
	uint32_t chipid_cidr;
	const char *name;

	unsigned n_gpnvms;
#define SAM3_N_NVM_BITS 3
	unsigned gpnvm[SAM3_N_NVM_BITS];
	unsigned total_flash_size;
	unsigned total_sram_size;
	unsigned n_banks;
#define SAM3_MAX_FLASH_BANKS 2
	/* these are "initialized" from the global const data */
	struct sam3_bank_private bank[SAM3_MAX_FLASH_BANKS];
};

struct sam3_chip {
	struct sam3_chip *next;
	bool probed;

	/* this is "initialized" from the global const structure */
	struct sam3_chip_details details;
	struct target *target;
	struct sam3_cfg cfg;
};


struct sam3_reg_list {
	uint32_t address;  size_t struct_offset; const char *name;
	void (*explain_func)(struct sam3_chip *chip);
};

static struct sam3_chip *all_sam3_chips;

static struct sam3_chip *get_current_sam3(struct command_invocation *cmd)
{
	struct target *t;
	static struct sam3_chip *p;

	t = get_current_target(cmd->ctx);
	if (!t) {
		command_print_sameline(cmd, "No current target?\n");
		return NULL;
	}

	p = all_sam3_chips;
	if (!p) {
		/* this should not happen */
		/* the command is not registered until the chip is created? */
		command_print_sameline(cmd, "No SAM3 chips exist?\n");
		return NULL;
	}

	while (p) {
		if (p->target == t)
			return p;
		p = p->next;
	}
	command_print_sameline(cmd, "Cannot find SAM3 chip?\n");
	return NULL;
}

/* these are used to *initialize* the "chip->details" structure. */
static const struct sam3_chip_details all_sam3_details[] = {
	/* Start at91sam3u* series */
	{
		.chipid_cidr    = 0x28100960,
		.name           = "at91sam3u4e",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 52 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_U,
				.controller_address = 0x400e0800,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 16,
				.sector_size = 8192,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_U,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 16,
				.sector_size = 8192,
				.page_size   = 256,
			},
		},
	},

	{
		.chipid_cidr    = 0x281a0760,
		.name           = "at91sam3u2e",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 36 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_U,
				.controller_address = 0x400e0800,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 16,
				.sector_size = 8192,
				.page_size   = 256,
			},
/*		  .bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},
	{
		.chipid_cidr    = 0x28190560,
		.name           = "at91sam3u1e",
		.total_flash_size     = 64 * 1024,
		.total_sram_size      = 20 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_U,
				.controller_address = 0x400e0800,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 64 * 1024,
				.nsectors   = 8,
				.sector_size = 8192,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x28000960,
		.name           = "at91sam3u4c",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 52 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
		{
			{
/*		.bank[0] = { */
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_U,
				.controller_address = 0x400e0800,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 16,
				.sector_size = 8192,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_U,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 16,
				.sector_size = 8192,
				.page_size   = 256,
			},
		},
	},

	{
		.chipid_cidr    = 0x280a0760,
		.name           = "at91sam3u2c",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 36 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_U,
				.controller_address = 0x400e0800,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 16,
				.sector_size = 8192,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},
	{
		.chipid_cidr    = 0x28090560,
		.name           = "at91sam3u1c",
		.total_flash_size     = 64 * 1024,
		.total_sram_size      = 20 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */

		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_U,
				.controller_address = 0x400e0800,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 64 * 1024,
				.nsectors   = 8,
				.sector_size = 8192,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},

	/* Start at91sam3s* series */

	/* Note: The preliminary at91sam3s datasheet says on page 302 */
	/* that the flash controller is at address 0x400E0800. */
	/* This is _not_ the case, the controller resides at address 0x400e0a00. */
	{
		.chipid_cidr    = 0x28A00960,
		.name           = "at91sam3s4c",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 48 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 1024,
				.nsectors   = 16,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},

	{
		.chipid_cidr    = 0x28900960,
		.name           = "at91sam3s4b",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 48 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 1024,
				.nsectors   = 16,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x28800960,
		.name           = "at91sam3s4a",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 48 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 1024,
				.nsectors   = 16,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x28AA0760,
		.name           = "at91sam3s2c",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 32 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 8,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x289A0760,
		.name           = "at91sam3s2b",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 32 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 8,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x298B0A60,
		.name           = "at91sam3sd8a",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*			.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes =  256 * 1024,
				.nsectors   =  16,
				.sector_size = 32768,
				.page_size   = 256,
			  },
/*			.bank[1] = { */
			  {
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_512K_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes =  256 * 1024,
				.nsectors   =  16,
				.sector_size = 32768,
				.page_size   = 256,
			},
		},
	},
	{
		.chipid_cidr    = 0x299B0A60,
		.name           = "at91sam3sd8b",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*			.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes =  256 * 1024,
				.nsectors   =  16,
				.sector_size = 32768,
				.page_size   = 256,
			  },
/*			.bank[1] = { */
			  {
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_512K_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes =  256 * 1024,
				.nsectors   =  16,
				.sector_size = 32768,
				.page_size   = 256,
			},
		},
	},
	{
		.chipid_cidr    = 0x29ab0a60,
		.name           = "at91sam3sd8c",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*			.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes =  256 * 1024,
				.nsectors   =  16,
				.sector_size = 32768,
				.page_size   = 256,
			  },
/*			.bank[1] = { */
			  {
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_512K_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes =  256 * 1024,
				.nsectors   =  16,
				.sector_size = 32768,
				.page_size   = 256,
			},
		},
	},
	{
		.chipid_cidr    = 0x288A0760,
		.name           = "at91sam3s2a",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 32 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 8,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x28A90560,
		.name           = "at91sam3s1c",
		.total_flash_size     = 64 * 1024,
		.total_sram_size      = 16 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 64 * 1024,
				.nsectors   = 4,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x28990560,
		.name           = "at91sam3s1b",
		.total_flash_size     = 64 * 1024,
		.total_sram_size      = 16 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 64 * 1024,
				.nsectors   = 4,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x28890560,
		.name           = "at91sam3s1a",
		.total_flash_size     = 64 * 1024,
		.total_sram_size      = 16 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 64 * 1024,
				.nsectors   = 4,
				.sector_size = 16384,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x288B0A60,
		.name           = "at91sam3s8a",
		.total_flash_size     = 256 * 2048,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 2048,
				.nsectors   = 16,
				.sector_size = 32768,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x289B0A60,
		.name           = "at91sam3s8b",
		.total_flash_size     = 256 * 2048,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 2048,
				.nsectors   = 16,
				.sector_size = 32768,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},
	{
		.chipid_cidr    = 0x28AB0A60,
		.name           = "at91sam3s8c",
		.total_flash_size     = 256 * 2048,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = { */
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 2048,
				.nsectors   = 16,
				.sector_size = 32768,
				.page_size   = 256,
			},
/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,

			},
		},
	},

	/* Start at91sam3n* series */
	{
		.chipid_cidr    = 0x29540960,
		.name           = "at91sam3n4c",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 24 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 1024,
				.nsectors   = 16,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29440960,
		.name           = "at91sam3n4b",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 24 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 1024,
				.nsectors   = 16,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29340960,
		.name           = "at91sam3n4a",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 24 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 256 * 1024,
				.nsectors   = 16,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29590760,
		.name           = "at91sam3n2c",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 16 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 8,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29490760,
		.name           = "at91sam3n2b",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 16 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 8,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29390760,
		.name           = "at91sam3n2a",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 16 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 128 * 1024,
				.nsectors   = 8,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29580560,
		.name           = "at91sam3n1c",
		.total_flash_size     = 64 * 1024,
		.total_sram_size      = 8 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 64 * 1024,
				.nsectors   = 4,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29480560,
		.name           = "at91sam3n1b",
		.total_flash_size     = 64 * 1024,
		.total_sram_size      = 8 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 64 * 1024,
				.nsectors   = 4,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29380560,
		.name           = "at91sam3n1a",
		.total_flash_size     = 64 * 1024,
		.total_sram_size      = 8 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

		/* System boots at address 0x0 */
		/* gpnvm[1] = selects boot code */
		/*     if gpnvm[1] == 0 */
		/*         boot is via "SAMBA" (rom) */
		/*     else */
		/*         boot is via FLASH */
		/*         Selection is via gpnvm[2] */
		/*     endif */
		/*  */
		/* NOTE: banks 0 & 1 switch places */
		/*     if gpnvm[2] == 0 */
		/*         Bank0 is the boot rom */
		/*      else */
		/*         Bank1 is the boot rom */
		/*      endif */
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 64 * 1024,
				.nsectors   = 4,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29480360,
		.name           = "at91sam3n0b",
		.total_flash_size     = 32 * 1024,
		.total_sram_size      = 8 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 32 * 1024,
				.nsectors   = 2,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29380360,
		.name           = "at91sam3n0a",
		.total_flash_size     = 32 * 1024,
		.total_sram_size      = 8 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 32 * 1024,
				.nsectors   = 2,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29450260,
		.name           = "at91sam3n00b",
		.total_flash_size     = 16 * 1024,
		.total_sram_size      = 4 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 16 * 1024,
				.nsectors   = 1,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},

	{
		.chipid_cidr    = 0x29350260,
		.name           = "at91sam3n00a",
		.total_flash_size     = 16 * 1024,
		.total_sram_size      = 4 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 1,

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.chip  = NULL,
				.bank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_N,
				.controller_address = 0x400e0A00,
				.flash_wait_states = 6,	/* workaround silicon bug */
				.present = 1,
				.size_bytes = 16 * 1024,
				.nsectors   = 1,
				.sector_size = 16384,
				.page_size   = 256,
			},

/*		.bank[1] = { */
			{
				.present = 0,
				.probed = false,
				.bank_number = 1,
			},
		},
	},


	/* Start at91sam3a series*/
	/* System boots at address 0x0 */
	/* gpnvm[1] = selects boot code */
	/*     if gpnvm[1] == 0 */
	/*         boot is via "SAMBA" (rom) */
	/*     else */
	/*         boot is via FLASH */
	/*         Selection is via gpnvm[2] */
	/*     endif */
	/*  */
	/* NOTE: banks 0 & 1 switch places */
	/*     if gpnvm[2] == 0 */
	/*         Bank0 is the boot rom */
	/*      else */
	/*         Bank1 is the boot rom */
	/*      endif */

	{
		.chipid_cidr    = 0x283E0A60,
		.name           = "at91sam3a8c",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 96 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*		.bank[0] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK0_BASE_AX,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,
		  },
/*		.bank[1] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 1,
			.base_address = FLASH_BANK1_BASE_512K_AX,
			.controller_address = 0x400e0c00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,

		  },
		},
	},
	{
		.chipid_cidr    = 0x283B0960,
		.name           = "at91sam3a4c",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*		.bank[0] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK0_BASE_AX,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  128 * 1024,
			.nsectors   =  8,
			.sector_size = 16384,
			.page_size   = 256,
		  },
/*		.bank[1] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 1,
			.base_address = FLASH_BANK1_BASE_256K_AX,
			.controller_address = 0x400e0c00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  128 * 1024,
			.nsectors   =  8,
			.sector_size = 16384,
			.page_size   = 256,

		  },
		},
	},

	/* Start at91sam3x* series */
	/* System boots at address 0x0 */
	/* gpnvm[1] = selects boot code */
	/*     if gpnvm[1] == 0 */
	/*         boot is via "SAMBA" (rom) */
	/*     else */
	/*         boot is via FLASH */
	/*         Selection is via gpnvm[2] */
	/*     endif */
	/*  */
	/* NOTE: banks 0 & 1 switch places */
	/*     if gpnvm[2] == 0 */
	/*         Bank0 is the boot rom */
	/*      else */
	/*         Bank1 is the boot rom */
	/*      endif */
	/*at91sam3x8h - ES has an incorrect CIDR of 0x286E0A20*/
	{
		.chipid_cidr    = 0x286E0A20,
		.name           = "at91sam3x8h - ES",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 96 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*		.bank[0] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK0_BASE_AX,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,
		  },
/*		.bank[1] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 1,
			.base_address = FLASH_BANK1_BASE_512K_AX,
			.controller_address = 0x400e0c00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,

		  },
		},
	},
	/*at91sam3x8h - ES2 and up uses the correct CIDR of 0x286E0A60*/
	{
		.chipid_cidr    = 0x286E0A60,
		.name           = "at91sam3x8h",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 96 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*		.bank[0] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK0_BASE_AX,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,
		  },
/*		.bank[1] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 1,
			.base_address = FLASH_BANK1_BASE_512K_AX,
			.controller_address = 0x400e0c00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,

		  },
		},
	},
	{
		.chipid_cidr    = 0x285E0A60,
		.name           = "at91sam3x8e",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 96 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*		.bank[0] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK0_BASE_AX,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,
		  },
/*		.bank[1] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 1,
			.base_address = FLASH_BANK1_BASE_512K_AX,
			.controller_address = 0x400e0c00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,

		  },
		},
	},
	{
		.chipid_cidr    = 0x284E0A60,
		.name           = "at91sam3x8c",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 96 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*		.bank[0] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK0_BASE_AX,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,
		  },
/*		.bank[1] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 1,
			.base_address = FLASH_BANK1_BASE_512K_AX,
			.controller_address = 0x400e0c00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  256 * 1024,
			.nsectors   =  16,
			.sector_size = 16384,
			.page_size   = 256,

		  },
		},
	},
	{
		.chipid_cidr    = 0x285B0960,
		.name           = "at91sam3x4e",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*		.bank[0] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK0_BASE_AX,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  128 * 1024,
			.nsectors   =  8,
			.sector_size = 16384,
			.page_size   = 256,
		  },
/*		.bank[1] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 1,
			.base_address = FLASH_BANK1_BASE_256K_AX,
			.controller_address = 0x400e0c00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  128 * 1024,
			.nsectors   =  8,
			.sector_size = 16384,
			.page_size   = 256,

		  },
		},
	},
	{
		.chipid_cidr    = 0x284B0960,
		.name           = "at91sam3x4c",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
		{
/*		.bank[0] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK0_BASE_AX,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  128 * 1024,
			.nsectors   =  8,
			.sector_size = 16384,
			.page_size   = 256,
		  },
/*		.bank[1] = { */
		  {
			.probed = false,
			.chip  = NULL,
			.bank  = NULL,
			.bank_number = 1,
			.base_address = FLASH_BANK1_BASE_256K_AX,
			.controller_address = 0x400e0c00,
			.flash_wait_states = 6,	/* workaround silicon bug */
			.present = 1,
			.size_bytes =  128 * 1024,
			.nsectors   =  8,
			.sector_size = 16384,
			.page_size   = 256,

		  },
		},
	},
	/* terminate */
	{
		.chipid_cidr    = 0,
		.name                   = NULL,
	}
};

/* Globals above */
/***********************************************************************
 **********************************************************************
 **********************************************************************
 **********************************************************************
 **********************************************************************
 **********************************************************************/
/* *ATMEL* style code - from the SAM3 driver code */

/**
 * Get the current status of the EEFC and
 * the value of some status bits (LOCKE, PROGE).
 * @param private  - info about the bank
 * @param v        - result goes here
 */
static int efc_get_status(struct sam3_bank_private *private, uint32_t *v)
{
	int r;
	r = target_read_u32(private->chip->target,
			private->controller_address + OFFSET_EFC_FSR,
			v);
	LOG_DEBUG("Status: 0x%08x (lockerror: %d, cmderror: %d, ready: %d)",
		(unsigned int)(*v),
		((unsigned int)((*v >> 2) & 1)),
		((unsigned int)((*v >> 1) & 1)),
		((unsigned int)((*v >> 0) & 1)));

	return r;
}

/**
 * Get the result of the last executed command.
 * @param private  - info about the bank
 * @param v        - result goes here
 */
static int efc_get_result(struct sam3_bank_private *private, uint32_t *v)
{
	int r;
	uint32_t rv;
	r = target_read_u32(private->chip->target,
			private->controller_address + OFFSET_EFC_FRR,
			&rv);
	if (v)
		*v = rv;
	LOG_DEBUG("Result: 0x%08x", ((unsigned int)(rv)));
	return r;
}

static int efc_start_command(struct sam3_bank_private *private,
	unsigned command, unsigned argument)
{
	uint32_t n, v;
	int r;
	int retry;

	retry = 0;
do_retry:

	/* Check command & argument */
	switch (command) {

		case AT91C_EFC_FCMD_WP:
		case AT91C_EFC_FCMD_WPL:
		case AT91C_EFC_FCMD_EWP:
		case AT91C_EFC_FCMD_EWPL:
		/* case AT91C_EFC_FCMD_EPL: */
		/* case AT91C_EFC_FCMD_EPA: */
		case AT91C_EFC_FCMD_SLB:
		case AT91C_EFC_FCMD_CLB:
			n = (private->size_bytes / private->page_size);
			if (argument >= n)
				LOG_ERROR("*BUG*: Embedded flash has only %u pages", (unsigned)(n));
			break;

		case AT91C_EFC_FCMD_SFB:
		case AT91C_EFC_FCMD_CFB:
			if (argument >= private->chip->details.n_gpnvms) {
				LOG_ERROR("*BUG*: Embedded flash has only %d GPNVMs",
						private->chip->details.n_gpnvms);
			}
			break;

		case AT91C_EFC_FCMD_GETD:
		case AT91C_EFC_FCMD_EA:
		case AT91C_EFC_FCMD_GLB:
		case AT91C_EFC_FCMD_GFB:
		case AT91C_EFC_FCMD_STUI:
		case AT91C_EFC_FCMD_SPUI:
			if (argument != 0)
				LOG_ERROR("Argument is meaningless for cmd: %d", command);
			break;
		default:
			LOG_ERROR("Unknown command %d", command);
			break;
	}

	if (command == AT91C_EFC_FCMD_SPUI) {
		/* this is a very special situation. */
		/* Situation (1) - error/retry - see below */
		/*      And we are being called recursively */
		/* Situation (2) - normal, finished reading unique id */
	} else {
		/* it should be "ready" */
		efc_get_status(private, &v);
		if (v & 1) {
			/* then it is ready */
			/* we go on */
		} else {
			if (retry) {
				/* we have done this before */
				/* the controller is not responding. */
				LOG_ERROR("flash controller(%d) is not ready! Error",
					private->bank_number);
				return ERROR_FAIL;
			} else {
				retry++;
				LOG_ERROR("Flash controller(%d) is not ready, attempting reset",
					private->bank_number);
				/* we do that by issuing the *STOP* command */
				efc_start_command(private, AT91C_EFC_FCMD_SPUI, 0);
				/* above is recursive, and further recursion is blocked by */
				/* if (command == AT91C_EFC_FCMD_SPUI) above */
				goto do_retry;
			}
		}
	}

	v = (0x5A << 24) | (argument << 8) | command;
	LOG_DEBUG("Command: 0x%08x", ((unsigned int)(v)));
	r = target_write_u32(private->bank->target,
			private->controller_address + OFFSET_EFC_FCR, v);
	if (r != ERROR_OK)
		LOG_DEBUG("Error Write failed");
	return r;
}

/**
 * Performs the given command and wait until its completion (or an error).
 * @param private  - info about the bank
 * @param command  - Command to perform.
 * @param argument - Optional command argument.
 * @param status   - put command status bits here
 */
static int efc_perform_command(struct sam3_bank_private *private,
	unsigned command,
	unsigned argument,
	uint32_t *status)
{

	int r;
	uint32_t v;
	int64_t ms_now, ms_end;

	/* default */
	if (status)
		*status = 0;

	r = efc_start_command(private, command, argument);
	if (r != ERROR_OK)
		return r;

	ms_end = 500 + timeval_ms();

	do {
		r = efc_get_status(private, &v);
		if (r != ERROR_OK)
			return r;
		ms_now = timeval_ms();
		if (ms_now > ms_end) {
			/* error */
			LOG_ERROR("Command timeout");
			return ERROR_FAIL;
		}
	} while ((v & 1) == 0);

	/* error bits.. */
	if (status)
		*status = (v & 0x6);
	return ERROR_OK;

}

/**
 * Read the unique ID.
 * @param private - info about the bank
 * The unique ID is stored in the 'private' structure.
 */
static int flashd_read_uid(struct sam3_bank_private *private)
{
	int r;
	uint32_t v;
	int x;
	/* assume 0 */
	private->chip->cfg.unique_id[0] = 0;
	private->chip->cfg.unique_id[1] = 0;
	private->chip->cfg.unique_id[2] = 0;
	private->chip->cfg.unique_id[3] = 0;

	LOG_DEBUG("Begin");
	r = efc_start_command(private, AT91C_EFC_FCMD_STUI, 0);
	if (r < 0)
		return r;

	for (x = 0; x < 4; x++) {
		r = target_read_u32(private->chip->target,
				private->bank->base + (x * 4),
				&v);
		if (r < 0)
			return r;
		private->chip->cfg.unique_id[x] = v;
	}

	r = efc_perform_command(private, AT91C_EFC_FCMD_SPUI, 0, NULL);
	LOG_DEBUG("End: R=%d, id = 0x%08x, 0x%08x, 0x%08x, 0x%08x",
		r,
		(unsigned int)(private->chip->cfg.unique_id[0]),
		(unsigned int)(private->chip->cfg.unique_id[1]),
		(unsigned int)(private->chip->cfg.unique_id[2]),
		(unsigned int)(private->chip->cfg.unique_id[3]));
	return r;

}

/**
 * Erases the entire flash.
 * @param private - the info about the bank.
 */
static int flashd_erase_entire_bank(struct sam3_bank_private *private)
{
	LOG_DEBUG("Here");
	return efc_perform_command(private, AT91C_EFC_FCMD_EA, 0, NULL);
}

/**
 * Gets current GPNVM state.
 * @param private  - info about the bank.
 * @param gpnvm    -  GPNVM bit index.
 * @param puthere  - result stored here.
 */
/* ------------------------------------------------------------------------------ */
static int flashd_get_gpnvm(struct sam3_bank_private *private, unsigned gpnvm, unsigned *puthere)
{
	uint32_t v;
	int r;

	LOG_DEBUG("Here");
	if (private->bank_number != 0) {
		LOG_ERROR("GPNVM only works with Bank0");
		return ERROR_FAIL;
	}

	if (gpnvm >= private->chip->details.n_gpnvms) {
		LOG_ERROR("Invalid GPNVM %d, max: %d, ignored",
			gpnvm, private->chip->details.n_gpnvms);
		return ERROR_FAIL;
	}

	/* Get GPNVMs status */
	r = efc_perform_command(private, AT91C_EFC_FCMD_GFB, 0, NULL);
	if (r != ERROR_OK) {
		LOG_ERROR("Failed");
		return r;
	}

	r = efc_get_result(private, &v);

	if (puthere) {
		/* Check if GPNVM is set */
		/* get the bit and make it a 0/1 */
		*puthere = (v >> gpnvm) & 1;
	}

	return r;
}

/**
 * Clears the selected GPNVM bit.
 * @param private info about the bank
 * @param gpnvm GPNVM index.
 * @returns 0 if successful; otherwise returns an error code.
 */
static int flashd_clr_gpnvm(struct sam3_bank_private *private, unsigned gpnvm)
{
	int r;
	unsigned v;

	LOG_DEBUG("Here");
	if (private->bank_number != 0) {
		LOG_ERROR("GPNVM only works with Bank0");
		return ERROR_FAIL;
	}

	if (gpnvm >= private->chip->details.n_gpnvms) {
		LOG_ERROR("Invalid GPNVM %d, max: %d, ignored",
			gpnvm, private->chip->details.n_gpnvms);
		return ERROR_FAIL;
	}

	r = flashd_get_gpnvm(private, gpnvm, &v);
	if (r != ERROR_OK) {
		LOG_DEBUG("Failed: %d", r);
		return r;
	}
	r = efc_perform_command(private, AT91C_EFC_FCMD_CFB, gpnvm, NULL);
	LOG_DEBUG("End: %d", r);
	return r;
}

/**
 * Sets the selected GPNVM bit.
 * @param private info about the bank
 * @param gpnvm GPNVM index.
 */
static int flashd_set_gpnvm(struct sam3_bank_private *private, unsigned gpnvm)
{
	int r;
	unsigned v;

	if (private->bank_number != 0) {
		LOG_ERROR("GPNVM only works with Bank0");
		return ERROR_FAIL;
	}

	if (gpnvm >= private->chip->details.n_gpnvms) {
		LOG_ERROR("Invalid GPNVM %d, max: %d, ignored",
			gpnvm, private->chip->details.n_gpnvms);
		return ERROR_FAIL;
	}

	r = flashd_get_gpnvm(private, gpnvm, &v);
	if (r != ERROR_OK)
		return r;
	if (v) {
		/* already set */
		r = ERROR_OK;
	} else {
		/* set it */
		r = efc_perform_command(private, AT91C_EFC_FCMD_SFB, gpnvm, NULL);
	}
	return r;
}

/**
 * Returns a bit field (at most 64) of locked regions within a page.
 * @param private info about the bank
 * @param v where to store locked bits
 */
static int flashd_get_lock_bits(struct sam3_bank_private *private, uint32_t *v)
{
	int r;
	LOG_DEBUG("Here");
	r = efc_perform_command(private, AT91C_EFC_FCMD_GLB, 0, NULL);
	if (r == ERROR_OK)
		r = efc_get_result(private, v);
	LOG_DEBUG("End: %d", r);
	return r;
}

/**
 * Unlocks all the regions in the given address range.
 * @param private info about the bank
 * @param start_sector first sector to unlock
 * @param end_sector last (inclusive) to unlock
 */

static int flashd_unlock(struct sam3_bank_private *private,
	unsigned start_sector,
	unsigned end_sector)
{
	int r;
	uint32_t status;
	uint32_t pg;
	uint32_t pages_per_sector;

	pages_per_sector = private->sector_size / private->page_size;

	/* Unlock all pages */
	while (start_sector <= end_sector) {
		pg = start_sector * pages_per_sector;

		r = efc_perform_command(private, AT91C_EFC_FCMD_CLB, pg, &status);
		if (r != ERROR_OK)
			return r;
		start_sector++;
	}

	return ERROR_OK;
}

/**
 * Locks regions
 * @param private - info about the bank
 * @param start_sector - first sector to lock
 * @param end_sector   - last sector (inclusive) to lock
 */
static int flashd_lock(struct sam3_bank_private *private,
	unsigned start_sector,
	unsigned end_sector)
{
	uint32_t status;
	uint32_t pg;
	uint32_t pages_per_sector;
	int r;

	pages_per_sector = private->sector_size / private->page_size;

	/* Lock all pages */
	while (start_sector <= end_sector) {
		pg = start_sector * pages_per_sector;

		r = efc_perform_command(private, AT91C_EFC_FCMD_SLB, pg, &status);
		if (r != ERROR_OK)
			return r;
		start_sector++;
	}
	return ERROR_OK;
}

/****** END SAM3 CODE ********/

/* begin helpful debug code */
/* print the fieldname, the field value, in dec & hex, and return field value */
static uint32_t sam3_reg_fieldname(struct sam3_chip *chip,
	const char *regname,
	uint32_t value,
	unsigned shift,
	unsigned width)
{
	uint32_t v;
	int hwidth, dwidth;


	/* extract the field */
	v = value >> shift;
	v = v & ((1 << width)-1);
	if (width <= 16) {
		hwidth = 4;
		dwidth = 5;
	} else {
		hwidth = 8;
		dwidth = 12;
	}

	/* show the basics */
	LOG_USER_N("\t%*s: %*" PRIu32 " [0x%0*" PRIx32 "] ",
		REG_NAME_WIDTH, regname,
		dwidth, v,
		hwidth, v);
	return v;
}

static const char _unknown[] = "unknown";
static const char *const eproc_names[] = {
	_unknown,					/* 0 */
	"arm946es",					/* 1 */
	"arm7tdmi",					/* 2 */
	"Cortex-M3",				/* 3 */
	"arm920t",					/* 4 */
	"arm926ejs",				/* 5 */
	_unknown,					/* 6 */
	_unknown,					/* 7 */
	_unknown,					/* 8 */
	_unknown,					/* 9 */
	_unknown,					/* 10 */
	_unknown,					/* 11 */
	_unknown,					/* 12 */
	_unknown,					/* 13 */
	_unknown,					/* 14 */
	_unknown,					/* 15 */
};

#define nvpsize2 nvpsize		/* these two tables are identical */
static const char *const nvpsize[] = {
	"none",						/*  0 */
	"8K bytes",					/*  1 */
	"16K bytes",				/*  2 */
	"32K bytes",				/*  3 */
	_unknown,					/*  4 */
	"64K bytes",				/*  5 */
	_unknown,					/*  6 */
	"128K bytes",				/*  7 */
	_unknown,					/*  8 */
	"256K bytes",				/*  9 */
	"512K bytes",				/* 10 */
	_unknown,					/* 11 */
	"1024K bytes",				/* 12 */
	_unknown,					/* 13 */
	"2048K bytes",				/* 14 */
	_unknown,					/* 15 */
};

static const char *const sramsize[] = {
	"48K Bytes",				/*  0 */
	"1K Bytes",					/*  1 */
	"2K Bytes",					/*  2 */
	"6K Bytes",					/*  3 */
	"112K Bytes",				/*  4 */
	"4K Bytes",					/*  5 */
	"80K Bytes",				/*  6 */
	"160K Bytes",				/*  7 */
	"8K Bytes",					/*  8 */
	"16K Bytes",				/*  9 */
	"32K Bytes",				/* 10 */
	"64K Bytes",				/* 11 */
	"128K Bytes",				/* 12 */
	"256K Bytes",				/* 13 */
	"96K Bytes",				/* 14 */
	"512K Bytes",				/* 15 */

};

static const struct archnames { unsigned value; const char *name; } archnames[] = {
	{ 0x19,  "AT91SAM9xx Series"                                            },
	{ 0x29,  "AT91SAM9XExx Series"                                          },
	{ 0x34,  "AT91x34 Series"                                                       },
	{ 0x37,  "CAP7 Series"                                                          },
	{ 0x39,  "CAP9 Series"                                                          },
	{ 0x3B,  "CAP11 Series"                                                         },
	{ 0x40,  "AT91x40 Series"                                                       },
	{ 0x42,  "AT91x42 Series"                                                       },
	{ 0x55,  "AT91x55 Series"                                                       },
	{ 0x60,  "AT91SAM7Axx Series"                                           },
	{ 0x61,  "AT91SAM7AQxx Series"                                          },
	{ 0x63,  "AT91x63 Series"                                                       },
	{ 0x70,  "AT91SAM7Sxx Series"                                           },
	{ 0x71,  "AT91SAM7XCxx Series"                                          },
	{ 0x72,  "AT91SAM7SExx Series"                                          },
	{ 0x73,  "AT91SAM7Lxx Series"                                           },
	{ 0x75,  "AT91SAM7Xxx Series"                                           },
	{ 0x76,  "AT91SAM7SLxx Series"                                          },
	{ 0x80,  "ATSAM3UxC Series (100-pin version)"           },
	{ 0x81,  "ATSAM3UxE Series (144-pin version)"           },
	{ 0x83,  "ATSAM3AxC Series (100-pin version)"           },
	{ 0x84,  "ATSAM3XxC Series (100-pin version)"           },
	{ 0x85,  "ATSAM3XxE Series (144-pin version)"           },
	{ 0x86,  "ATSAM3XxG Series (208/217-pin version)"       },
	{ 0x88,  "ATSAM3SxA Series (48-pin version)"            },
	{ 0x89,  "ATSAM3SxB Series (64-pin version)"            },
	{ 0x8A,  "ATSAM3SxC Series (100-pin version)"           },
	{ 0x92,  "AT91x92 Series"                                                       },
	{ 0x93,  "ATSAM3NxA Series (48-pin version)"            },
	{ 0x94,  "ATSAM3NxB Series (64-pin version)"            },
	{ 0x95,  "ATSAM3NxC Series (100-pin version)"           },
	{ 0x98,  "ATSAM3SDxA Series (48-pin version)"           },
	{ 0x99,  "ATSAM3SDxB Series (64-pin version)"           },
	{ 0x9A,  "ATSAM3SDxC Series (100-pin version)"          },
	{ 0xA5,  "ATSAM5A"                                                              },
	{ 0xF0,  "AT75Cxx Series"                                                       },
	{ -1, NULL },
};

static const char *const nvptype[] = {
	"rom",	/* 0 */
	"romless or onchip flash",	/* 1 */
	"embedded flash memory",/* 2 */
	"rom(nvpsiz) + embedded flash (nvpsiz2)",	/* 3 */
	"sram emulating flash",	/* 4 */
	_unknown,	/* 5 */
	_unknown,	/* 6 */
	_unknown,	/* 7 */
};

static const char *_yes_or_no(uint32_t v)
{
	if (v)
		return "YES";
	else
		return "NO";
}

static const char *const _rc_freq[] = {
	"4 MHz", "8 MHz", "12 MHz", "reserved"
};

static void sam3_explain_ckgr_mor(struct sam3_chip *chip)
{
	uint32_t v;
	uint32_t rcen;

	v = sam3_reg_fieldname(chip, "MOSCXTEN", chip->cfg.CKGR_MOR, 0, 1);
	LOG_USER("(main xtal enabled: %s)", _yes_or_no(v));
	v = sam3_reg_fieldname(chip, "MOSCXTBY", chip->cfg.CKGR_MOR, 1, 1);
	LOG_USER("(main osc bypass: %s)", _yes_or_no(v));
	rcen = sam3_reg_fieldname(chip, "MOSCRCEN", chip->cfg.CKGR_MOR, 3, 1);
	LOG_USER("(onchip RC-OSC enabled: %s)", _yes_or_no(rcen));
	v = sam3_reg_fieldname(chip, "MOSCRCF", chip->cfg.CKGR_MOR, 4, 3);
	LOG_USER("(onchip RC-OSC freq: %s)", _rc_freq[v]);

	chip->cfg.rc_freq = 0;
	if (rcen) {
		switch (v) {
			default:
				chip->cfg.rc_freq = 0;
				break;
			case 0:
				chip->cfg.rc_freq = 4 * 1000 * 1000;
				break;
			case 1:
				chip->cfg.rc_freq = 8 * 1000 * 1000;
				break;
			case 2:
				chip->cfg.rc_freq = 12 * 1000 * 1000;
				break;
		}
	}

	v = sam3_reg_fieldname(chip, "MOSCXTST", chip->cfg.CKGR_MOR, 8, 8);
	LOG_USER("(startup clks, time= %f uSecs)",
		((float)(v * 1000000)) / ((float)(chip->cfg.slow_freq)));
	v = sam3_reg_fieldname(chip, "MOSCSEL", chip->cfg.CKGR_MOR, 24, 1);
	LOG_USER("(mainosc source: %s)",
		v ? "external xtal" : "internal RC");

	v = sam3_reg_fieldname(chip, "CFDEN", chip->cfg.CKGR_MOR, 25, 1);
	LOG_USER("(clock failure enabled: %s)",
		_yes_or_no(v));
}

static void sam3_explain_chipid_cidr(struct sam3_chip *chip)
{
	int x;
	uint32_t v;
	const char *cp;

	sam3_reg_fieldname(chip, "Version", chip->cfg.CHIPID_CIDR, 0, 5);
	LOG_USER_N("\n");

	v = sam3_reg_fieldname(chip, "EPROC", chip->cfg.CHIPID_CIDR, 5, 3);
	LOG_USER("%s", eproc_names[v]);

	v = sam3_reg_fieldname(chip, "NVPSIZE", chip->cfg.CHIPID_CIDR, 8, 4);
	LOG_USER("%s", nvpsize[v]);

	v = sam3_reg_fieldname(chip, "NVPSIZE2", chip->cfg.CHIPID_CIDR, 12, 4);
	LOG_USER("%s", nvpsize2[v]);

	v = sam3_reg_fieldname(chip, "SRAMSIZE", chip->cfg.CHIPID_CIDR, 16, 4);
	LOG_USER("%s", sramsize[v]);

	v = sam3_reg_fieldname(chip, "ARCH", chip->cfg.CHIPID_CIDR, 20, 8);
	cp = _unknown;
	for (x = 0; archnames[x].name; x++) {
		if (v == archnames[x].value) {
			cp = archnames[x].name;
			break;
		}
	}

	LOG_USER("%s", cp);

	v = sam3_reg_fieldname(chip, "NVPTYP", chip->cfg.CHIPID_CIDR, 28, 3);
	LOG_USER("%s", nvptype[v]);

	v = sam3_reg_fieldname(chip, "EXTID", chip->cfg.CHIPID_CIDR, 31, 1);
	LOG_USER("(exists: %s)", _yes_or_no(v));
}

static void sam3_explain_ckgr_mcfr(struct sam3_chip *chip)
{
	uint32_t v;

	v = sam3_reg_fieldname(chip, "MAINFRDY", chip->cfg.CKGR_MCFR, 16, 1);
	LOG_USER("(main ready: %s)", _yes_or_no(v));

	v = sam3_reg_fieldname(chip, "MAINF", chip->cfg.CKGR_MCFR, 0, 16);

	v = (v * chip->cfg.slow_freq) / 16;
	chip->cfg.mainosc_freq = v;

	LOG_USER("(%3.03f Mhz (%" PRIu32 ".%03" PRIu32 "khz slowclk)",
		_tomhz(v),
		(uint32_t)(chip->cfg.slow_freq / 1000),
		(uint32_t)(chip->cfg.slow_freq % 1000));
}

static void sam3_explain_ckgr_plla(struct sam3_chip *chip)
{
	uint32_t mula, diva;

	diva = sam3_reg_fieldname(chip, "DIVA", chip->cfg.CKGR_PLLAR, 0, 8);
	LOG_USER_N("\n");
	mula = sam3_reg_fieldname(chip, "MULA", chip->cfg.CKGR_PLLAR, 16, 11);
	LOG_USER_N("\n");
	chip->cfg.plla_freq = 0;
	if (mula == 0)
		LOG_USER("\tPLLA Freq: (Disabled,mula = 0)");
	else if (diva == 0)
		LOG_USER("\tPLLA Freq: (Disabled,diva = 0)");
	else if (diva >= 1) {
		chip->cfg.plla_freq = (chip->cfg.mainosc_freq * (mula + 1) / diva);
		LOG_USER("\tPLLA Freq: %3.03f MHz",
			_tomhz(chip->cfg.plla_freq));
	}
}

static void sam3_explain_mckr(struct sam3_chip *chip)
{
	uint32_t css, pres, fin = 0;
	int pdiv = 0;
	const char *cp = NULL;

	css = sam3_reg_fieldname(chip, "CSS", chip->cfg.PMC_MCKR, 0, 2);
	switch (css & 3) {
		case 0:
			fin = chip->cfg.slow_freq;
			cp = "slowclk";
			break;
		case 1:
			fin = chip->cfg.mainosc_freq;
			cp  = "mainosc";
			break;
		case 2:
			fin = chip->cfg.plla_freq;
			cp  = "plla";
			break;
		case 3:
			if (chip->cfg.CKGR_UCKR & (1 << 16)) {
				fin = 480 * 1000 * 1000;
				cp = "upll";
			} else {
				fin = 0;
				cp  = "upll (*ERROR* UPLL is disabled)";
			}
			break;
		default:
			assert(0);
			break;
	}

	LOG_USER("%s (%3.03f Mhz)",
		cp,
		_tomhz(fin));
	pres = sam3_reg_fieldname(chip, "PRES", chip->cfg.PMC_MCKR, 4, 3);
	switch (pres & 0x07) {
		case 0:
			pdiv = 1;
			cp = "selected clock";
			break;
		case 1:
			pdiv = 2;
			cp = "clock/2";
			break;
		case 2:
			pdiv = 4;
			cp = "clock/4";
			break;
		case 3:
			pdiv = 8;
			cp = "clock/8";
			break;
		case 4:
			pdiv = 16;
			cp = "clock/16";
			break;
		case 5:
			pdiv = 32;
			cp = "clock/32";
			break;
		case 6:
			pdiv = 64;
			cp = "clock/64";
			break;
		case 7:
			pdiv = 6;
			cp = "clock/6";
			break;
		default:
			assert(0);
			break;
	}
	LOG_USER("(%s)", cp);
	fin = fin / pdiv;
	/* sam3 has a *SINGLE* clock - */
	/* other at91 series parts have divisors for these. */
	chip->cfg.cpu_freq = fin;
	chip->cfg.mclk_freq = fin;
	chip->cfg.fclk_freq = fin;
	LOG_USER("\t\tResult CPU Freq: %3.03f",
		_tomhz(fin));
}

#if 0
static struct sam3_chip *target2sam3(struct target *target)
{
	struct sam3_chip *chip;

	if (!target)
		return NULL;

	chip = all_sam3_chips;
	while (chip) {
		if (chip->target == target)
			break;	/* return below */
		else
			chip = chip->next;
	}
	return chip;
}
#endif

static uint32_t *sam3_get_reg_ptr(struct sam3_cfg *cfg, const struct sam3_reg_list *list)
{
	/* this function exists to help */
	/* keep funky offsetof() errors */
	/* and casting from causing bugs */

	/* By using prototypes - we can detect what would */
	/* be casting errors. */

	return (uint32_t *)(void *)(((char *)(cfg)) + list->struct_offset);
}


#define SAM3_ENTRY(NAME, FUNC)  { .address = SAM3_ ## NAME, .struct_offset = offsetof( \
						  struct sam3_cfg, \
						  NAME), # NAME, FUNC }
static const struct sam3_reg_list sam3_all_regs[] = {
	SAM3_ENTRY(CKGR_MOR, sam3_explain_ckgr_mor),
	SAM3_ENTRY(CKGR_MCFR, sam3_explain_ckgr_mcfr),
	SAM3_ENTRY(CKGR_PLLAR, sam3_explain_ckgr_plla),
	SAM3_ENTRY(CKGR_UCKR, NULL),
	SAM3_ENTRY(PMC_FSMR, NULL),
	SAM3_ENTRY(PMC_FSPR, NULL),
	SAM3_ENTRY(PMC_IMR, NULL),
	SAM3_ENTRY(PMC_MCKR, sam3_explain_mckr),
	SAM3_ENTRY(PMC_PCK0, NULL),
	SAM3_ENTRY(PMC_PCK1, NULL),
	SAM3_ENTRY(PMC_PCK2, NULL),
	SAM3_ENTRY(PMC_PCSR, NULL),
	SAM3_ENTRY(PMC_SCSR, NULL),
	SAM3_ENTRY(PMC_SR, NULL),
	SAM3_ENTRY(CHIPID_CIDR, sam3_explain_chipid_cidr),
	SAM3_ENTRY(CHIPID_CIDR2, sam3_explain_chipid_cidr),
	SAM3_ENTRY(CHIPID_EXID, NULL),
	SAM3_ENTRY(CHIPID_EXID2, NULL),
	/* TERMINATE THE LIST */
	{ .name = NULL }
};
#undef SAM3_ENTRY

static struct sam3_bank_private *get_sam3_bank_private(struct flash_bank *bank)
{
	return bank->driver_priv;
}

/**
 * Given a pointer to where it goes in the structure,
 * determine the register name, address from the all registers table.
 */
static const struct sam3_reg_list *sam3_get_reg(struct sam3_chip *chip, uint32_t *goes_here)
{
	const struct sam3_reg_list *reg;

	reg = &(sam3_all_regs[0]);
	while (reg->name) {
		uint32_t *possible;

		/* calculate where this one go.. */
		/* it is "possibly" this register. */

		possible = ((uint32_t *)(void *)(((char *)(&(chip->cfg))) + reg->struct_offset));

		/* well? Is it this register */
		if (possible == goes_here) {
			/* Jump for joy! */
			return reg;
		}

		/* next... */
		reg++;
	}
	/* This is *TOTAL*PANIC* - we are totally screwed. */
	LOG_ERROR("INVALID SAM3 REGISTER");
	return NULL;
}

static int sam3_read_this_reg(struct sam3_chip *chip, uint32_t *goes_here)
{
	const struct sam3_reg_list *reg;
	int r;

	reg = sam3_get_reg(chip, goes_here);
	if (!reg)
		return ERROR_FAIL;

	r = target_read_u32(chip->target, reg->address, goes_here);
	if (r != ERROR_OK) {
		LOG_ERROR("Cannot read SAM3 register: %s @ 0x%08x, Err: %d",
			reg->name, (unsigned)(reg->address), r);
	}
	return r;
}

static int sam3_read_all_regs(struct sam3_chip *chip)
{
	int r;
	const struct sam3_reg_list *reg;

	reg = &(sam3_all_regs[0]);
	while (reg->name) {
		r = sam3_read_this_reg(chip,
				sam3_get_reg_ptr(&(chip->cfg), reg));
		if (r != ERROR_OK) {
			LOG_ERROR("Cannot read SAM3 register: %s @ 0x%08x, Error: %d",
				reg->name, ((unsigned)(reg->address)), r);
			return r;
		}
		reg++;
	}

	/* Chip identification register
	*
	* Unfortunately, the chip identification register is not at
	* a constant address across all of the SAM3 series'. As a
	* consequence, a simple heuristic is used to find where it's
	* at...
	*
	* If the contents at the first address is zero, then we know
	* that the second address is where the chip id register is.
	* We can deduce this because for those SAM's that have the
	* chip id @ 0x400e0940, the first address, 0x400e0740, is
	* located in the memory map of the Power Management Controller
	* (PMC). Furthermore, the address is not used by the PMC.
	* So when read, the memory controller returns zero.*/
	if (chip->cfg.CHIPID_CIDR == 0)	{
		/*Put the correct CIDR and EXID values in the chip structure */
		chip->cfg.CHIPID_CIDR = chip->cfg.CHIPID_CIDR2;
		chip->cfg.CHIPID_EXID = chip->cfg.CHIPID_EXID2;
	}
	return ERROR_OK;
}

static int sam3_get_info(struct sam3_chip *chip)
{
	const struct sam3_reg_list *reg;
	uint32_t regval;

	reg = &(sam3_all_regs[0]);
	while (reg->name) {
		/* display all regs */
		LOG_DEBUG("Start: %s", reg->name);
		regval = *sam3_get_reg_ptr(&(chip->cfg), reg);
		LOG_USER("%*s: [0x%08" PRIx32 "] -> 0x%08" PRIx32,
			REG_NAME_WIDTH,
			reg->name,
			reg->address,
			regval);
		if (reg->explain_func)
			(*(reg->explain_func))(chip);
		LOG_DEBUG("End: %s", reg->name);
		reg++;
	}
	LOG_USER("   rc-osc: %3.03f MHz", _tomhz(chip->cfg.rc_freq));
	LOG_USER("  mainosc: %3.03f MHz", _tomhz(chip->cfg.mainosc_freq));
	LOG_USER("     plla: %3.03f MHz", _tomhz(chip->cfg.plla_freq));
	LOG_USER(" cpu-freq: %3.03f MHz", _tomhz(chip->cfg.cpu_freq));
	LOG_USER("mclk-freq: %3.03f MHz", _tomhz(chip->cfg.mclk_freq));

	LOG_USER(" UniqueId: 0x%08" PRIx32 " 0x%08" PRIx32 " 0x%08" PRIx32 " 0x%08" PRIx32,
		chip->cfg.unique_id[0],
		chip->cfg.unique_id[1],
		chip->cfg.unique_id[2],
		chip->cfg.unique_id[3]);

	return ERROR_OK;
}

static int sam3_protect_check(struct flash_bank *bank)
{
	int r;
	uint32_t v = 0;
	unsigned x;
	struct sam3_bank_private *private;

	LOG_DEBUG("Begin");
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	private = get_sam3_bank_private(bank);
	if (!private) {
		LOG_ERROR("no private for this bank?");
		return ERROR_FAIL;
	}
	if (!(private->probed))
		return ERROR_FLASH_BANK_NOT_PROBED;

	r = flashd_get_lock_bits(private, &v);
	if (r != ERROR_OK) {
		LOG_DEBUG("Failed: %d", r);
		return r;
	}

	for (x = 0; x < private->nsectors; x++)
		bank->sectors[x].is_protected = (!!(v & (1 << x)));
	LOG_DEBUG("Done");
	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(sam3_flash_bank_command)
{
	struct sam3_chip *chip;

	chip = all_sam3_chips;

	/* is this an existing chip? */
	while (chip) {
		if (chip->target == bank->target)
			break;
		chip = chip->next;
	}

	if (!chip) {
		/* this is a *NEW* chip */
		chip = calloc(1, sizeof(struct sam3_chip));
		if (!chip) {
			LOG_ERROR("NO RAM!");
			return ERROR_FAIL;
		}
		chip->target = bank->target;
		/* insert at head */
		chip->next = all_sam3_chips;
		all_sam3_chips = chip;
		chip->target = bank->target;
		/* assumption is this runs at 32khz */
		chip->cfg.slow_freq = 32768;
		chip->probed = false;
	}

	switch (bank->base) {
		default:
			LOG_ERROR("Address 0x%08x invalid bank address (try 0x%08x or 0x%08x "
			"[at91sam3u series] or 0x%08x [at91sam3s series] or "
			"0x%08x [at91sam3n series] or 0x%08x or 0x%08x or 0x%08x[at91sam3ax series] )",
			((unsigned int)(bank->base)),
			((unsigned int)(FLASH_BANK0_BASE_U)),
			((unsigned int)(FLASH_BANK1_BASE_U)),
			((unsigned int)(FLASH_BANK_BASE_S)),
			((unsigned int)(FLASH_BANK_BASE_N)),
			((unsigned int)(FLASH_BANK0_BASE_AX)),
		    ((unsigned int)(FLASH_BANK1_BASE_256K_AX)),
		    ((unsigned int)(FLASH_BANK1_BASE_512K_AX)));
			return ERROR_FAIL;

		/* at91sam3s and at91sam3n series only has bank 0*/
		/* at91sam3u and at91sam3ax series has the same address for bank 0*/
		case FLASH_BANK_BASE_S:
		case FLASH_BANK0_BASE_U:
			bank->driver_priv = &(chip->details.bank[0]);
			bank->bank_number = 0;
			chip->details.bank[0].chip = chip;
			chip->details.bank[0].bank = bank;
			break;

		/* Bank 1 of at91sam3u or at91sam3ax series */
		case FLASH_BANK1_BASE_U:
		case FLASH_BANK1_BASE_256K_AX:
		case FLASH_BANK1_BASE_512K_AX:
			bank->driver_priv = &(chip->details.bank[1]);
			bank->bank_number = 1;
			chip->details.bank[1].chip = chip;
			chip->details.bank[1].bank = bank;
			break;
	}

	/* we initialize after probing. */
	return ERROR_OK;
}

/**
 * Remove all chips from the internal list without distinguishing which one
 * is owned by this bank. This simplification works only for one shot
 * deallocation like current flash_free_all_banks()
 */
static void sam3_free_driver_priv(struct flash_bank *bank)
{
	struct sam3_chip *chip = all_sam3_chips;
	while (chip) {
		struct sam3_chip *next = chip->next;
		free(chip);
		chip = next;
	}
	all_sam3_chips = NULL;
}

static int sam3_get_details(struct sam3_bank_private *private)
{
	const struct sam3_chip_details *details;
	struct sam3_chip *chip;
	struct flash_bank *saved_banks[SAM3_MAX_FLASH_BANKS];
	unsigned x;

	LOG_DEBUG("Begin");
	details = all_sam3_details;
	while (details->name) {
		/* Compare cidr without version bits */
		if (((details->chipid_cidr ^ private->chip->cfg.CHIPID_CIDR) & 0xFFFFFFE0) == 0)
			break;
		else
			details++;
	}
	if (!details->name) {
		LOG_ERROR("SAM3 ChipID 0x%08x not found in table (perhaps you can ID this chip?)",
			(unsigned int)(private->chip->cfg.CHIPID_CIDR));
		/* Help the victim, print details about the chip */
		LOG_INFO("SAM3 CHIPID_CIDR: 0x%08" PRIx32 " decodes as follows",
			private->chip->cfg.CHIPID_CIDR);
		sam3_explain_chipid_cidr(private->chip);
		return ERROR_FAIL;
	}

	/* DANGER: THERE ARE DRAGONS HERE */

	/* get our chip - it is going */
	/* to be over-written shortly */
	chip = private->chip;

	/* Note that, in reality: */
	/*  */
	/*     private = &(chip->details.bank[0]) */
	/* or  private = &(chip->details.bank[1]) */
	/*  */

	/* save the "bank" pointers */
	for (x = 0; x < SAM3_MAX_FLASH_BANKS; x++)
		saved_banks[x] = chip->details.bank[x].bank;

	/* Overwrite the "details" structure. */
	memcpy(&(private->chip->details),
		details,
		sizeof(private->chip->details));

	/* now fix the ghosted pointers */
	for (x = 0; x < SAM3_MAX_FLASH_BANKS; x++) {
		chip->details.bank[x].chip = chip;
		chip->details.bank[x].bank = saved_banks[x];
	}

	/* update the *BANK*SIZE* */

	LOG_DEBUG("End");
	return ERROR_OK;
}

static int _sam3_probe(struct flash_bank *bank, int noise)
{
	int r;
	struct sam3_bank_private *private;


	LOG_DEBUG("Begin: Bank: %u, Noise: %d", bank->bank_number, noise);
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	private = get_sam3_bank_private(bank);
	if (!private) {
		LOG_ERROR("Invalid/unknown bank number");
		return ERROR_FAIL;
	}

	r = sam3_read_all_regs(private->chip);
	if (r != ERROR_OK)
		return r;

	LOG_DEBUG("Here");
	if (private->chip->probed)
		r = sam3_get_info(private->chip);
	else
		r = sam3_get_details(private);
	if (r != ERROR_OK)
		return r;

	/* update the flash bank size */
	for (unsigned int x = 0; x < SAM3_MAX_FLASH_BANKS; x++) {
		if (bank->base == private->chip->details.bank[x].base_address) {
			bank->size = private->chip->details.bank[x].size_bytes;
			break;
		}
	}

	if (!bank->sectors) {
		bank->sectors = calloc(private->nsectors, (sizeof((bank->sectors)[0])));
		if (!bank->sectors) {
			LOG_ERROR("No memory!");
			return ERROR_FAIL;
		}
		bank->num_sectors = private->nsectors;

		for (unsigned int x = 0; x < bank->num_sectors; x++) {
			bank->sectors[x].size = private->sector_size;
			bank->sectors[x].offset = x * (private->sector_size);
			/* mark as unknown */
			bank->sectors[x].is_erased = -1;
			bank->sectors[x].is_protected = -1;
		}
	}

	private->probed = true;

	r = sam3_protect_check(bank);
	if (r != ERROR_OK)
		return r;

	LOG_DEBUG("Bank = %d, nbanks = %d",
		private->bank_number, private->chip->details.n_banks);
	if ((private->bank_number + 1) == private->chip->details.n_banks) {
		/* read unique id, */
		/* it appears to be associated with the *last* flash bank. */
		flashd_read_uid(private);
	}

	return r;
}

static int sam3_probe(struct flash_bank *bank)
{
	return _sam3_probe(bank, 1);
}

static int sam3_auto_probe(struct flash_bank *bank)
{
	return _sam3_probe(bank, 0);
}

static int sam3_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct sam3_bank_private *private;
	int r;

	LOG_DEBUG("Here");
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	r = sam3_auto_probe(bank);
	if (r != ERROR_OK) {
		LOG_DEBUG("Here,r=%d", r);
		return r;
	}

	private = get_sam3_bank_private(bank);
	if (!(private->probed))
		return ERROR_FLASH_BANK_NOT_PROBED;

	if ((first == 0) && ((last + 1) == private->nsectors)) {
		/* whole chip */
		LOG_DEBUG("Here");
		return flashd_erase_entire_bank(private);
	}
	LOG_INFO("sam3 auto-erases while programming (request ignored)");
	return ERROR_OK;
}

static int sam3_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct sam3_bank_private *private;
	int r;

	LOG_DEBUG("Here");
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	private = get_sam3_bank_private(bank);
	if (!(private->probed))
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (set)
		r = flashd_lock(private, first, last);
	else
		r = flashd_unlock(private, first, last);
	LOG_DEBUG("End: r=%d", r);

	return r;

}

static int sam3_page_read(struct sam3_bank_private *private, unsigned pagenum, uint8_t *buf)
{
	uint32_t adr;
	int r;

	adr = pagenum * private->page_size;
	adr += private->base_address;

	r = target_read_memory(private->chip->target,
			adr,
			4,					/* THIS*MUST*BE* in 32bit values */
			private->page_size / 4,
			buf);
	if (r != ERROR_OK)
		LOG_ERROR("SAM3: Flash program failed to read page phys address: 0x%08x",
			(unsigned int)(adr));
	return r;
}

static int sam3_page_write(struct sam3_bank_private *private, unsigned pagenum, const uint8_t *buf)
{
	uint32_t adr;
	uint32_t status;
	uint32_t fmr;	/* EEFC Flash Mode Register */
	int r;

	adr = pagenum * private->page_size;
	adr += private->base_address;

	/* Get flash mode register value */
	r = target_read_u32(private->chip->target, private->controller_address, &fmr);
	if (r != ERROR_OK)
		LOG_DEBUG("Error Read failed: read flash mode register");

	/* Clear flash wait state field */
	fmr &= 0xfffff0ff;

	/* set FWS (flash wait states) field in the FMR (flash mode register) */
	fmr |= (private->flash_wait_states << 8);

	LOG_DEBUG("Flash Mode: 0x%08x", ((unsigned int)(fmr)));
	r = target_write_u32(private->bank->target, private->controller_address, fmr);
	if (r != ERROR_OK)
		LOG_DEBUG("Error Write failed: set flash mode register");

	LOG_DEBUG("Wr Page %u @ phys address: 0x%08x", pagenum, (unsigned int)(adr));
	r = target_write_memory(private->chip->target,
			adr,
			4,					/* THIS*MUST*BE* in 32bit values */
			private->page_size / 4,
			buf);
	if (r != ERROR_OK) {
		LOG_ERROR("SAM3: Failed to write (buffer) page at phys address 0x%08x",
			(unsigned int)(adr));
		return r;
	}

	r = efc_perform_command(private,
			/* send Erase & Write Page */
			AT91C_EFC_FCMD_EWP,
			pagenum,
			&status);

	if (r != ERROR_OK)
		LOG_ERROR("SAM3: Error performing Erase & Write page @ phys address 0x%08x",
			(unsigned int)(adr));
	if (status & (1 << 2)) {
		LOG_ERROR("SAM3: Page @ Phys address 0x%08x is locked", (unsigned int)(adr));
		return ERROR_FAIL;
	}
	if (status & (1 << 1)) {
		LOG_ERROR("SAM3: Flash Command error @phys address 0x%08x", (unsigned int)(adr));
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int sam3_write(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	int n;
	unsigned page_cur;
	unsigned page_end;
	int r;
	unsigned page_offset;
	struct sam3_bank_private *private;
	uint8_t *pagebuffer;

	/* in case we bail further below, set this to null */
	pagebuffer = NULL;

	/* ignore dumb requests */
	if (count == 0) {
		r = ERROR_OK;
		goto done;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		r = ERROR_TARGET_NOT_HALTED;
		goto done;
	}

	private = get_sam3_bank_private(bank);
	if (!(private->probed)) {
		r = ERROR_FLASH_BANK_NOT_PROBED;
		goto done;
	}

	if ((offset + count) > private->size_bytes) {
		LOG_ERROR("Flash write error - past end of bank");
		LOG_ERROR(" offset: 0x%08x, count 0x%08x, BankEnd: 0x%08x",
			(unsigned int)(offset),
			(unsigned int)(count),
			(unsigned int)(private->size_bytes));
		r = ERROR_FAIL;
		goto done;
	}

	pagebuffer = malloc(private->page_size);
	if (!pagebuffer) {
		LOG_ERROR("No memory for %d Byte page buffer", (int)(private->page_size));
		r = ERROR_FAIL;
		goto done;
	}

	/* what page do we start & end in? */
	page_cur = offset / private->page_size;
	page_end = (offset + count - 1) / private->page_size;

	LOG_DEBUG("Offset: 0x%08x, Count: 0x%08x", (unsigned int)(offset), (unsigned int)(count));
	LOG_DEBUG("Page start: %d, Page End: %d", (int)(page_cur), (int)(page_end));

	/* Special case: all one page */
	/*  */
	/* Otherwise: */
	/*    (1) non-aligned start */
	/*    (2) body pages */
	/*    (3) non-aligned end. */

	/* Handle special case - all one page. */
	if (page_cur == page_end) {
		LOG_DEBUG("Special case, all in one page");
		r = sam3_page_read(private, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;

		page_offset = (offset & (private->page_size-1));
		memcpy(pagebuffer + page_offset,
			buffer,
			count);

		r = sam3_page_write(private, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;
		r = ERROR_OK;
		goto done;
	}

	/* non-aligned start */
	page_offset = offset & (private->page_size - 1);
	if (page_offset) {
		LOG_DEBUG("Not-Aligned start");
		/* read the partial */
		r = sam3_page_read(private, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;

		/* over-write with new data */
		n = (private->page_size - page_offset);
		memcpy(pagebuffer + page_offset,
			buffer,
			n);

		r = sam3_page_write(private, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;

		count  -= n;
		offset += n;
		buffer += n;
		page_cur++;
	}

	/* By checking that offset is correct here, we also
	fix a clang warning */
	assert(offset % private->page_size == 0);

	/* intermediate large pages */
	/* also - the final *terminal* */
	/* if that terminal page is a full page */
	LOG_DEBUG("Full Page Loop: cur=%d, end=%d, count = 0x%08x",
		(int)page_cur, (int)page_end, (unsigned int)(count));

	while ((page_cur < page_end) &&
			(count >= private->page_size)) {
		r = sam3_page_write(private, page_cur, buffer);
		if (r != ERROR_OK)
			goto done;
		count -= private->page_size;
		buffer += private->page_size;
		page_cur += 1;
	}

	/* terminal partial page? */
	if (count) {
		LOG_DEBUG("Terminal partial page, count = 0x%08x", (unsigned int)(count));
		/* we have a partial page */
		r = sam3_page_read(private, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;
					/* data goes at start */
		memcpy(pagebuffer, buffer, count);
		r = sam3_page_write(private, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;
	}
	LOG_DEBUG("Done!");
	r = ERROR_OK;
done:
	free(pagebuffer);
	return r;
}

COMMAND_HANDLER(sam3_handle_info_command)
{
	struct sam3_chip *chip;
	chip = get_current_sam3(CMD);
	if (!chip)
		return ERROR_OK;

	unsigned x;
	int r;

	/* bank0 must exist before we can do anything */
	if (!chip->details.bank[0].bank) {
		x = 0;
need_define:
		command_print(CMD,
			"Please define bank %d via command: flash bank %s ... ",
			x,
			at91sam3_flash.name);
		return ERROR_FAIL;
	}

	/* if bank 0 is not probed, then probe it */
	if (!(chip->details.bank[0].probed)) {
		r = sam3_auto_probe(chip->details.bank[0].bank);
		if (r != ERROR_OK)
			return ERROR_FAIL;
	}
	/* above guarantees the "chip details" structure is valid */
	/* and thus, bank private areas are valid */
	/* and we have a SAM3 chip, what a concept! */

	/* auto-probe other banks, 0 done above */
	for (x = 1; x < SAM3_MAX_FLASH_BANKS; x++) {
		/* skip banks not present */
		if (!(chip->details.bank[x].present))
			continue;

		if (!chip->details.bank[x].bank)
			goto need_define;

		if (chip->details.bank[x].probed)
			continue;

		r = sam3_auto_probe(chip->details.bank[x].bank);
		if (r != ERROR_OK)
			return r;
	}

	r = sam3_get_info(chip);
	if (r != ERROR_OK) {
		LOG_DEBUG("Sam3Info, Failed %d", r);
		return r;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(sam3_handle_gpnvm_command)
{
	unsigned x, v;
	int r, who;
	struct sam3_chip *chip;

	chip = get_current_sam3(CMD);
	if (!chip)
		return ERROR_OK;

	if (chip->target->state != TARGET_HALTED) {
		LOG_ERROR("sam3 - target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->details.bank[0].bank) {
		command_print(CMD, "Bank0 must be defined first via: flash bank %s ...",
			at91sam3_flash.name);
		return ERROR_FAIL;
	}
	if (!chip->details.bank[0].probed) {
		r = sam3_auto_probe(chip->details.bank[0].bank);
		if (r != ERROR_OK)
			return r;
	}

	switch (CMD_ARGC) {
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
		case 0:
			goto showall;
		case 1:
			who = -1;
			break;
		case 2:
			if ((strcmp(CMD_ARGV[0], "show") == 0) && (strcmp(CMD_ARGV[1], "all") == 0))
				who = -1;
			else {
				uint32_t v32;
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], v32);
				who = v32;
			}
			break;
	}

	if (strcmp("show", CMD_ARGV[0]) == 0) {
		if (who == -1) {
showall:
			r = ERROR_OK;
			for (x = 0; x < chip->details.n_gpnvms; x++) {
				r = flashd_get_gpnvm(&(chip->details.bank[0]), x, &v);
				if (r != ERROR_OK)
					break;
				command_print(CMD, "sam3-gpnvm%u: %u", x, v);
			}
			return r;
		}
		if ((who >= 0) && (((unsigned)(who)) < chip->details.n_gpnvms)) {
			r = flashd_get_gpnvm(&(chip->details.bank[0]), who, &v);
			if (r == ERROR_OK)
				command_print(CMD, "sam3-gpnvm%u: %u", who, v);
			return r;
		} else {
			command_print(CMD, "sam3-gpnvm invalid GPNVM: %u", who);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (who == -1) {
		command_print(CMD, "Missing GPNVM number");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (strcmp("set", CMD_ARGV[0]) == 0)
		r = flashd_set_gpnvm(&(chip->details.bank[0]), who);
	else if ((strcmp("clr", CMD_ARGV[0]) == 0) ||
		 (strcmp("clear", CMD_ARGV[0]) == 0))			/* quietly accept both */
		r = flashd_clr_gpnvm(&(chip->details.bank[0]), who);
	else {
		command_print(CMD, "Unknown command: %s", CMD_ARGV[0]);
		r = ERROR_COMMAND_SYNTAX_ERROR;
	}
	return r;
}

COMMAND_HANDLER(sam3_handle_slowclk_command)
{
	struct sam3_chip *chip;

	chip = get_current_sam3(CMD);
	if (!chip)
		return ERROR_OK;

	switch (CMD_ARGC) {
		case 0:
			/* show */
			break;
		case 1:
		{
			/* set */
			uint32_t v;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], v);
			if (v > 200000) {
				/* absurd slow clock of 200Khz? */
				command_print(CMD, "Absurd/illegal slow clock freq: %d\n", (int)(v));
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			chip->cfg.slow_freq = v;
			break;
		}
		default:
			/* error */
			command_print(CMD, "Too many parameters");
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	command_print(CMD, "Slowclk freq: %d.%03dkhz",
		(int)(chip->cfg.slow_freq / 1000),
		(int)(chip->cfg.slow_freq % 1000));
	return ERROR_OK;
}

static const struct command_registration at91sam3_exec_command_handlers[] = {
	{
		.name = "gpnvm",
		.handler = sam3_handle_gpnvm_command,
		.mode = COMMAND_EXEC,
		.usage = "[('clr'|'set'|'show') bitnum]",
		.help = "Without arguments, shows all bits in the gpnvm "
			"register.  Otherwise, clears, sets, or shows one "
			"General Purpose Non-Volatile Memory (gpnvm) bit.",
	},
	{
		.name = "info",
		.handler = sam3_handle_info_command,
		.mode = COMMAND_EXEC,
		.help = "Print information about the current at91sam3 chip "
			"and its flash configuration.",
		.usage = "",
	},
	{
		.name = "slowclk",
		.handler = sam3_handle_slowclk_command,
		.mode = COMMAND_EXEC,
		.usage = "[clock_hz]",
		.help = "Display or set the slowclock frequency "
			"(default 32768 Hz).",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration at91sam3_command_handlers[] = {
	{
		.name = "at91sam3",
		.mode = COMMAND_ANY,
		.help = "at91sam3 flash command group",
		.usage = "",
		.chain = at91sam3_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver at91sam3_flash = {
	.name = "at91sam3",
	.commands = at91sam3_command_handlers,
	.flash_bank_command = sam3_flash_bank_command,
	.erase = sam3_erase,
	.protect = sam3_protect,
	.write = sam3_write,
	.read = default_flash_read,
	.probe = sam3_probe,
	.auto_probe = sam3_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = sam3_protect_check,
	.free_driver_priv = sam3_free_driver_priv,
};
