/***************************************************************************
 *   Copyright (C) 2009 by Duane Ellis                                     *
 *   openocd@duaneellis.com                                                *
 *                                                                         *
 *   Copyright (C) 2010 by Olaf Lüke (at91sam3s* support)                  *
 *   olaf@uni-paderborn.de                                                 *
 *                                                                         *
 *   Copyright (C) 2011 by Olivier Schonken, Jim Norris                    *
 *   (at91sam3x* & at91sam4 support)*                                      *
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

/* at91sam4s/at91sam4e/at91sam4c series (has always one flash bank)*/
#define FLASH_BANK_BASE_S   0x00400000
#define FLASH_BANK_BASE_C   0x01000000

/* at91sam4sd series (two one flash banks), first bank address */
#define FLASH_BANK0_BASE_SD FLASH_BANK_BASE_S
/* at91sam4sd16x, second bank address */
#define FLASH_BANK1_BASE_1024K_SD (FLASH_BANK0_BASE_SD+(1024*1024/2))
/* at91sam4sd32x, second bank address */
#define FLASH_BANK1_BASE_2048K_SD (FLASH_BANK0_BASE_SD+(2048*1024/2))

/* at91sam4c32x, first and second bank address */
#define FLASH_BANK0_BASE_C32 FLASH_BANK_BASE_C
#define FLASH_BANK1_BASE_C32 (FLASH_BANK_BASE_C+(2048*1024/2))

#define         AT91C_EFC_FCMD_GETD                 (0x0)	/* (EFC) Get Flash Descriptor */
#define         AT91C_EFC_FCMD_WP                   (0x1)	/* (EFC) Write Page */
#define         AT91C_EFC_FCMD_WPL                  (0x2)	/* (EFC) Write Page and Lock */
#define         AT91C_EFC_FCMD_EWP                  (0x3)	/* (EFC) Erase Page and Write Page */
#define         AT91C_EFC_FCMD_EWPL                 (0x4)	/* (EFC) Erase Page and Write Page then Lock */
#define         AT91C_EFC_FCMD_EA                   (0x5)	/* (EFC) Erase All */
/* cmd6 is not present in the at91sam4u4/2/1 data sheet table 19-2 */
/* #define      AT91C_EFC_FCMD_EPL                  (0x6) // (EFC) Erase plane? */
#define			AT91C_EFC_FCMD_EPA                  (0x7)     /* (EFC) Erase pages */
#define         AT91C_EFC_FCMD_SLB                  (0x8)	/* (EFC) Set Lock Bit */
#define         AT91C_EFC_FCMD_CLB                  (0x9)	/* (EFC) Clear Lock Bit */
#define         AT91C_EFC_FCMD_GLB                  (0xA)	/* (EFC) Get Lock Bit */
#define         AT91C_EFC_FCMD_SFB                  (0xB)	/* (EFC) Set Fuse Bit */
#define         AT91C_EFC_FCMD_CFB                  (0xC)	/* (EFC) Clear Fuse Bit */
#define         AT91C_EFC_FCMD_GFB                  (0xD)	/* (EFC) Get Fuse Bit */
#define         AT91C_EFC_FCMD_STUI                 (0xE)	/* (EFC) Start Read Unique ID */
#define         AT91C_EFC_FCMD_SPUI                 (0xF)	/* (EFC) Stop Read Unique ID */

#define  offset_EFC_FMR   0
#define  offset_EFC_FCR   4
#define  offset_EFC_FSR   8
#define  offset_EFC_FRR   12

extern const struct flash_driver at91sam4_flash;

static float _tomhz(uint32_t freq_hz)
{
	float f;

	f = ((float)(freq_hz)) / 1000000.0;
	return f;
}

/* How the chip is configured. */
struct sam4_cfg {
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


#define SAM4_CHIPID_CIDR          (0x400E0740)
	uint32_t CHIPID_CIDR;
#define SAM4_CHIPID_EXID          (0x400E0744)
	uint32_t CHIPID_EXID;

#define SAM4_PMC_BASE             (0x400E0400)
#define SAM4_PMC_SCSR             (SAM4_PMC_BASE + 0x0008)
	uint32_t PMC_SCSR;
#define SAM4_PMC_PCSR             (SAM4_PMC_BASE + 0x0018)
	uint32_t PMC_PCSR;
#define SAM4_CKGR_UCKR            (SAM4_PMC_BASE + 0x001c)
	uint32_t CKGR_UCKR;
#define SAM4_CKGR_MOR             (SAM4_PMC_BASE + 0x0020)
	uint32_t CKGR_MOR;
#define SAM4_CKGR_MCFR            (SAM4_PMC_BASE + 0x0024)
	uint32_t CKGR_MCFR;
#define SAM4_CKGR_PLLAR           (SAM4_PMC_BASE + 0x0028)
	uint32_t CKGR_PLLAR;
#define SAM4_PMC_MCKR             (SAM4_PMC_BASE + 0x0030)
	uint32_t PMC_MCKR;
#define SAM4_PMC_PCK0             (SAM4_PMC_BASE + 0x0040)
	uint32_t PMC_PCK0;
#define SAM4_PMC_PCK1             (SAM4_PMC_BASE + 0x0044)
	uint32_t PMC_PCK1;
#define SAM4_PMC_PCK2             (SAM4_PMC_BASE + 0x0048)
	uint32_t PMC_PCK2;
#define SAM4_PMC_SR               (SAM4_PMC_BASE + 0x0068)
	uint32_t PMC_SR;
#define SAM4_PMC_IMR              (SAM4_PMC_BASE + 0x006c)
	uint32_t PMC_IMR;
#define SAM4_PMC_FSMR             (SAM4_PMC_BASE + 0x0070)
	uint32_t PMC_FSMR;
#define SAM4_PMC_FSPR             (SAM4_PMC_BASE + 0x0074)
	uint32_t PMC_FSPR;
};

struct sam4_bank_private {
	bool probed;
	/* DANGER: THERE ARE DRAGONS HERE.. */
	/* NOTE: If you add more 'ghost' pointers */
	/* be aware that you must *manually* update */
	/* these pointers in the function sam4_GetDetails() */
	/* See the comment "Here there be dragons" */

	/* so we can find the chip we belong to */
	struct sam4_chip *pChip;
	/* so we can find the original bank pointer */
	struct flash_bank *pBank;
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

struct sam4_chip_details {
	/* THERE ARE DRAGONS HERE.. */
	/* note: If you add pointers here */
	/* be careful about them as they */
	/* may need to be updated inside */
	/* the function: "sam4_GetDetails() */
	/* which copy/overwrites the */
	/* 'runtime' copy of this structure */
	uint32_t chipid_cidr;
	const char *name;

	unsigned n_gpnvms;
#define SAM4_N_NVM_BITS 3
	unsigned gpnvm[SAM4_N_NVM_BITS];
	unsigned total_flash_size;
	unsigned total_sram_size;
	unsigned n_banks;
#define SAM4_MAX_FLASH_BANKS 2
	/* these are "initialized" from the global const data */
	struct sam4_bank_private bank[SAM4_MAX_FLASH_BANKS];
};

struct sam4_chip {
	struct sam4_chip *next;
	bool probed;

	/* this is "initialized" from the global const structure */
	struct sam4_chip_details details;
	struct target *target;
	struct sam4_cfg cfg;
};


struct sam4_reg_list {
	uint32_t address;  size_t struct_offset; const char *name;
	void (*explain_func)(struct sam4_chip *pInfo);
};

static struct sam4_chip *all_sam4_chips;

static struct sam4_chip *get_current_sam4(struct command_invocation *cmd)
{
	struct target *t;
	static struct sam4_chip *p;

	t = get_current_target(cmd->ctx);
	if (!t) {
		command_print(cmd, "No current target?");
		return NULL;
	}

	p = all_sam4_chips;
	if (!p) {
		/* this should not happen */
		/* the command is not registered until the chip is created? */
		command_print(cmd, "No SAM4 chips exist?");
		return NULL;
	}

	while (p) {
		if (p->target == t)
			return p;
		p = p->next;
	}
	command_print(cmd, "Cannot find SAM4 chip?");
	return NULL;
}

/*The actual sector size of the SAM4S flash memory is 65536 bytes. 16 sectors for a 1024KB device*/
/*The lockregions are 8KB per lock region, with a 1024KB device having 128 lock regions. */
/*For the best results, nsectors are thus set to the amount of lock regions, and the sector_size*/
/*set to the lock region size.  Page erases are used to erase 8KB sections when programming*/

/* these are used to *initialize* the "pChip->details" structure. */
static const struct sam4_chip_details all_sam4_details[] = {
	/* Start at91sam4c* series */
	/* at91sam4c32e - LQFP144 */
	{
		.chipid_cidr    = 0xA66D0EE0,
		.name           = "at91sam4c32e",
		.total_flash_size     = 2024 * 1024,
		.total_sram_size      = 256 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_C32,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  1024 * 1024,
				.nsectors   =  128,
				.sector_size = 8192,
				.page_size   = 512,
			},
/*		.bank[1] = { */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_C32,
				.controller_address = 0x400e0c00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  1024 * 1024,
				.nsectors   =  128,
				.sector_size = 8192,
				.page_size   = 512,
			},
		},
	},
	/* at91sam4c32c - LQFP100 */
	{
		.chipid_cidr    = 0xA64D0EE0,
		.name           = "at91sam4c32c",
		.total_flash_size     = 2024 * 1024,
		.total_sram_size      = 256 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,
/*		.bank[0] = { */
		{
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_C32,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  1024 * 1024,
				.nsectors   =  128,
				.sector_size = 8192,
				.page_size   = 512,
			},
/*		.bank[1] = { */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_C32,
				.controller_address = 0x400e0c00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  1024 * 1024,
				.nsectors   =  128,
				.sector_size = 8192,
				.page_size   = 512,
			},
		},
	},
	/* at91sam4c16c - LQFP100 */
	{
		.chipid_cidr    = 0xA64C0CE0,
		.name           = "at91sam4c16c",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_C,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/* at91sam4c8c - LQFP100 */
	{
		.chipid_cidr    = 0xA64C0AE0,
		.name           = "at91sam4c8c",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_C,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  512 * 1024,
			.nsectors   =  64,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/* at91sam4c4c (rev B) - LQFP100 */
	{
		.chipid_cidr    = 0xA64C0CE5,
		.name           = "at91sam4c4c",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_C,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  256 * 1024,
			.nsectors   =  32,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/* Start at91sam4e* series */
	/*atsam4e16e - LQFP144/LFBGA144*/
	{
		.chipid_cidr    = 0xA3CC0CE0,
		.name           = "at91sam4e16e",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/* Start at91sam4n* series */
	/*atsam4n8a - LQFP48/QFN48*/
	{
		.chipid_cidr    = 0x293B0AE0,
		.name           = "at91sam4n8a",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  512 * 1024,
			.nsectors   =  64,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4n8b - LQFP64/QFN64*/
	{
		.chipid_cidr    = 0x294B0AE0,
		.name           = "at91sam4n8b",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  512 * 1024,
			.nsectors   =  64,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4n8c - LQFP100/TFBGA100/VFBGA100*/
	{
		.chipid_cidr    = 0x295B0AE0,
		.name           = "at91sam4n8c",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  512 * 1024,
			.nsectors   =  64,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4n16b - LQFP64/QFN64*/
	{
		.chipid_cidr    = 0x29460CE0,
		.name           = "at91sam4n16b",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 80 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4n16c - LQFP100/TFBGA100/VFBGA100*/
	{
		.chipid_cidr    = 0x29560CE0,
		.name           = "at91sam4n16c",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 80 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/* Start at91sam4s* series */
	/*atsam4s16c - LQFP100/BGA100*/
	{
		.chipid_cidr    = 0x28AC0CE0,
		.name           = "at91sam4s16c",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*at91sam4sa16c - TFBGA100/VFBGA100/LQFP100*/
	{
		.chipid_cidr    = 0x28a70ce0,
		.name           = "at91sam4sa16c",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

/*		.bank[0] = { */
		{
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4s16b - LQFP64/QFN64/WLCSP64*/
	{
		.chipid_cidr    = 0x289C0CE0,
		.name           = "at91sam4s16b",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4sa16b - LQFP64/QFN64*/
	{
		.chipid_cidr    = 0x28970CE0,
		.name           = "at91sam4sa16b",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4s16a - LQFP48/QFN48*/
	{
		.chipid_cidr    = 0x288C0CE0,
		.name           = "at91sam4s16a",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  1024 * 1024,
			.nsectors   =  128,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4s8c - LQFP100/BGA100*/
	{
		.chipid_cidr    = 0x28AC0AE0,
		.name           = "at91sam4s8c",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  512 * 1024,
			.nsectors   =  64,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4s8b - LQFP64/QFN64/WLCSP64*/
	{
		.chipid_cidr    = 0x289C0AE0,
		.name           = "at91sam4s8b",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  512 * 1024,
			.nsectors   =  64,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},
	/*atsam4s8a - LQFP48/BGA48*/
	{
		.chipid_cidr    = 0x288C0AE0,
		.name           = "at91sam4s8a",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 128 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  512 * 1024,
			.nsectors   =  64,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/*atsam4s4c - LQFP100/BGA100*/
	{
		.chipid_cidr    = 0x28ab09e0,
		.name           = "at91sam4s4c",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  256 * 1024,
			.nsectors   =  32,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/*atsam4s4b - LQFP64/QFN64/WLCSP64*/
	{
		.chipid_cidr    = 0x289b09e0,
		.name           = "at91sam4s4b",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  256 * 1024,
			.nsectors   =  32,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/*atsam4s4a - LQFP48/QFN48*/
	{
		.chipid_cidr    = 0x288b09e0,
		.name           = "at91sam4s4a",
		.total_flash_size     = 256 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  256 * 1024,
			.nsectors   =  32,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/*atsam4s2c - LQFP100/BGA100*/
	{
		.chipid_cidr    = 0x28ab07e0,
		.name           = "at91sam4s2c",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  128 * 1024,
			.nsectors   =  16,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/*atsam4s2b - LQPF64/QFN64/WLCSP64*/
	{
		.chipid_cidr    = 0x289b07e0,
		.name           = "at91sam4s2b",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  128 * 1024,
			.nsectors   =  16,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/*atsam4s2a - LQFP48/QFN48*/
	{
		.chipid_cidr    = 0x288b07e0,
		.name           = "at91sam4s2a",
		.total_flash_size     = 128 * 1024,
		.total_sram_size      = 64 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,
		{
/*		.bank[0] = {*/
		  {
			.probed = false,
			.pChip  = NULL,
			.pBank  = NULL,
			.bank_number = 0,
			.base_address = FLASH_BANK_BASE_S,
			.controller_address = 0x400e0a00,
			.flash_wait_states = 5,
			.present = true,
			.size_bytes =  128 * 1024,
			.nsectors   =  16,
			.sector_size = 8192,
			.page_size   = 512,
		  },
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		},
	},

	/*at91sam4sd32c  - LQFP100/BGA100*/
	{
		.chipid_cidr    = 0x29a70ee0,
		.name           = "at91sam4sd32c",
		.total_flash_size     = 2048 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  1024 * 1024,
				.nsectors   =  128,
				.sector_size = 8192,
				.page_size   = 512,
			},

/*		.bank[1] = { */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_2048K_SD,
				.controller_address = 0x400e0c00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  1024 * 1024,
				.nsectors   =  128,
				.sector_size = 8192,
				.page_size   = 512,
			},
		},
	},

	/*at91sam4sd32b  - LQFP64/BGA64*/
	{
		.chipid_cidr    = 0x29970ee0,
		.name           = "at91sam4sd32b",
		.total_flash_size     = 2048 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  1024 * 1024,
				.nsectors   =  128,
				.sector_size = 8192,
				.page_size   = 512,
			},

/*		.bank[1] = { */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_2048K_SD,
				.controller_address = 0x400e0c00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  1024 * 1024,
				.nsectors   =  128,
				.sector_size = 8192,
				.page_size   = 512,
			},
		},
	},

	/*at91sam4sd16c - LQFP100/BGA100*/
	{
		.chipid_cidr    = 0x29a70ce0,
		.name           = "at91sam4sd16c",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},

/*		.bank[1] = { */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_1024K_SD,
				.controller_address = 0x400e0c00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},
		},
	},

	/*at91sam4sd16b - LQFP64/BGA64*/
	{
		.chipid_cidr    = 0x29970ce0,
		.name           = "at91sam4sd16b",
		.total_flash_size     = 1024 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 3,
		.n_banks        = 2,

/*		.bank[0] = { */
		{
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK0_BASE_SD,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},

/*		.bank[1] = { */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 1,
				.base_address = FLASH_BANK1_BASE_1024K_SD,
				.controller_address = 0x400e0c00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},
		},
	},

	/* atsamg53n19 */
	{
		.chipid_cidr    = 0x247e0ae0,
		.name           = "atsamg53n19",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 96 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

/*		.bank[0] = {*/
		{
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},
/*		.bank[1] = {*/
		  {
			.present = false,
			.probed = false,
			.bank_number = 1,

		  },
		}
	},

	/* atsamg55g19 Rev.A */
	{
		.chipid_cidr    = 0x24470ae0,
		.name           = "atsamg55g19",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

		{
/*			.bank[0] = */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},
/*			.bank[1] = */
			{
				.present = false,
				.probed = false,
				.bank_number = 1,
			},
		}
	},

	/* atsamg55g19 Rev.B */
	{
		.chipid_cidr    = 0x24470ae1,
		.name           = "atsamg55g19b",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

		{
/*			.bank[0] = */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},
/*			.bank[1] = */
			{
				.present = false,
				.probed = false,
				.bank_number = 1,
			},
		}
	},

	/* atsamg55j19 Rev.A */
	{
		.chipid_cidr    = 0x24570ae0,
		.name           = "atsamg55j19",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

		{
/*			.bank[0] = */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},
/*			.bank[1] = */
			{
				.present = false,
				.probed = false,
				.bank_number = 1,
			},
		}
	},

	/* atsamg55j19 Rev.B */
	{
		.chipid_cidr    = 0x24570ae1,
		.name           = "atsamg55j19b",
		.total_flash_size     = 512 * 1024,
		.total_sram_size      = 160 * 1024,
		.n_gpnvms       = 2,
		.n_banks        = 1,

		{
/*			.bank[0] = */
			{
				.probed = false,
				.pChip  = NULL,
				.pBank  = NULL,
				.bank_number = 0,
				.base_address = FLASH_BANK_BASE_S,
				.controller_address = 0x400e0a00,
				.flash_wait_states = 5,
				.present = true,
				.size_bytes =  512 * 1024,
				.nsectors   =  64,
				.sector_size = 8192,
				.page_size   = 512,
			},
/*			.bank[1] = */
			{
				.present = false,
				.probed = false,
				.bank_number = 1,
			},
		}
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
/* *ATMEL* style code - from the SAM4 driver code */

/**
 * Get the current status of the EEFC and
 * the value of some status bits (LOCKE, PROGE).
 * @param pPrivate - info about the bank
 * @param v        - result goes here
 */
static int EFC_GetStatus(struct sam4_bank_private *pPrivate, uint32_t *v)
{
	int r;
	r = target_read_u32(pPrivate->pChip->target,
			pPrivate->controller_address + offset_EFC_FSR,
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
 * @param pPrivate - info about the bank
 * @param v        - result goes here
 */
static int EFC_GetResult(struct sam4_bank_private *pPrivate, uint32_t *v)
{
	int r;
	uint32_t rv;
	r = target_read_u32(pPrivate->pChip->target,
			pPrivate->controller_address + offset_EFC_FRR,
			&rv);
	if (v)
		*v = rv;
	LOG_DEBUG("Result: 0x%08x", ((unsigned int)(rv)));
	return r;
}

static int EFC_StartCommand(struct sam4_bank_private *pPrivate,
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
		case AT91C_EFC_FCMD_EPA:
		case AT91C_EFC_FCMD_SLB:
		case AT91C_EFC_FCMD_CLB:
			n = (pPrivate->size_bytes / pPrivate->page_size);
			if (argument >= n)
				LOG_ERROR("*BUG*: Embedded flash has only %u pages", (unsigned)(n));
			break;

		case AT91C_EFC_FCMD_SFB:
		case AT91C_EFC_FCMD_CFB:
			if (argument >= pPrivate->pChip->details.n_gpnvms) {
				LOG_ERROR("*BUG*: Embedded flash has only %d GPNVMs",
						pPrivate->pChip->details.n_gpnvms);
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
		EFC_GetStatus(pPrivate, &v);
		if (v & 1) {
			/* then it is ready */
			/* we go on */
		} else {
			if (retry) {
				/* we have done this before */
				/* the controller is not responding. */
				LOG_ERROR("flash controller(%d) is not ready! Error",
					pPrivate->bank_number);
				return ERROR_FAIL;
			} else {
				retry++;
				LOG_ERROR("Flash controller(%d) is not ready, attempting reset",
					pPrivate->bank_number);
				/* we do that by issuing the *STOP* command */
				EFC_StartCommand(pPrivate, AT91C_EFC_FCMD_SPUI, 0);
				/* above is recursive, and further recursion is blocked by */
				/* if (command == AT91C_EFC_FCMD_SPUI) above */
				goto do_retry;
			}
		}
	}

	v = (0x5A << 24) | (argument << 8) | command;
	LOG_DEBUG("Command: 0x%08x", ((unsigned int)(v)));
	r = target_write_u32(pPrivate->pBank->target,
			pPrivate->controller_address + offset_EFC_FCR, v);
	if (r != ERROR_OK)
		LOG_DEBUG("Error Write failed");
	return r;
}

/**
 * Performs the given command and wait until its completion (or an error).
 * @param pPrivate - info about the bank
 * @param command  - Command to perform.
 * @param argument - Optional command argument.
 * @param status   - put command status bits here
 */
static int EFC_PerformCommand(struct sam4_bank_private *pPrivate,
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

	r = EFC_StartCommand(pPrivate, command, argument);
	if (r != ERROR_OK)
		return r;

	ms_end = 10000 + timeval_ms();

	do {
		r = EFC_GetStatus(pPrivate, &v);
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
 * @param pPrivate - info about the bank
 * The unique ID is stored in the 'pPrivate' structure.
 */
static int FLASHD_ReadUniqueID(struct sam4_bank_private *pPrivate)
{
	int r;
	uint32_t v;
	int x;
	/* assume 0 */
	pPrivate->pChip->cfg.unique_id[0] = 0;
	pPrivate->pChip->cfg.unique_id[1] = 0;
	pPrivate->pChip->cfg.unique_id[2] = 0;
	pPrivate->pChip->cfg.unique_id[3] = 0;

	LOG_DEBUG("Begin");
	r = EFC_StartCommand(pPrivate, AT91C_EFC_FCMD_STUI, 0);
	if (r < 0)
		return r;

	for (x = 0; x < 4; x++) {
		r = target_read_u32(pPrivate->pChip->target,
				pPrivate->pBank->base + (x * 4),
				&v);
		if (r < 0)
			return r;
		pPrivate->pChip->cfg.unique_id[x] = v;
	}

	r = EFC_PerformCommand(pPrivate, AT91C_EFC_FCMD_SPUI, 0, NULL);
	LOG_DEBUG("End: R=%d, id = 0x%08x, 0x%08x, 0x%08x, 0x%08x",
		r,
		(unsigned int)(pPrivate->pChip->cfg.unique_id[0]),
		(unsigned int)(pPrivate->pChip->cfg.unique_id[1]),
		(unsigned int)(pPrivate->pChip->cfg.unique_id[2]),
		(unsigned int)(pPrivate->pChip->cfg.unique_id[3]));
	return r;

}

/**
 * Erases the entire flash.
 * @param pPrivate - the info about the bank.
 */
static int FLASHD_EraseEntireBank(struct sam4_bank_private *pPrivate)
{
	LOG_DEBUG("Here");
	return EFC_PerformCommand(pPrivate, AT91C_EFC_FCMD_EA, 0, NULL);
}

/**
 * Erases the entire flash.
 * @param pPrivate - the info about the bank.
 * @param firstPage
 * @param numPages
 * @param status
 */
static int FLASHD_ErasePages(struct sam4_bank_private *pPrivate,
							 int firstPage,
							 int numPages,
							 uint32_t *status)
{
	LOG_DEBUG("Here");
	uint8_t erasePages;
	switch (numPages)	{
		case 4:
			erasePages = 0x00;
			break;
		case 8:
			erasePages = 0x01;
			break;
		case 16:
			erasePages = 0x02;
			break;
		case 32:
			erasePages = 0x03;
			break;
		default:
			erasePages = 0x00;
			break;
	}

	/* AT91C_EFC_FCMD_EPA
	 * According to the datasheet FARG[15:2] defines the page from which
	 * the erase will start.This page must be modulo 4, 8, 16 or 32
	 * according to the number of pages to erase. FARG[1:0] defines the
	 * number of pages to be erased. Previously (firstpage << 2) was used
	 * to conform to this, seems it should not be shifted...
	 */
	return EFC_PerformCommand(pPrivate,
		/* send Erase Page */
		AT91C_EFC_FCMD_EPA,
		(firstPage) | erasePages,
		status);
}

/**
 * Gets current GPNVM state.
 * @param pPrivate - info about the bank.
 * @param gpnvm    -  GPNVM bit index.
 * @param puthere  - result stored here.
 */
/* ------------------------------------------------------------------------------ */
static int FLASHD_GetGPNVM(struct sam4_bank_private *pPrivate, unsigned gpnvm, unsigned *puthere)
{
	uint32_t v;
	int r;

	LOG_DEBUG("Here");
	if (pPrivate->bank_number != 0) {
		LOG_ERROR("GPNVM only works with Bank0");
		return ERROR_FAIL;
	}

	if (gpnvm >= pPrivate->pChip->details.n_gpnvms) {
		LOG_ERROR("Invalid GPNVM %d, max: %d, ignored",
			gpnvm, pPrivate->pChip->details.n_gpnvms);
		return ERROR_FAIL;
	}

	/* Get GPNVMs status */
	r = EFC_PerformCommand(pPrivate, AT91C_EFC_FCMD_GFB, 0, NULL);
	if (r != ERROR_OK) {
		LOG_ERROR("Failed");
		return r;
	}

	r = EFC_GetResult(pPrivate, &v);

	if (puthere) {
		/* Check if GPNVM is set */
		/* get the bit and make it a 0/1 */
		*puthere = (v >> gpnvm) & 1;
	}

	return r;
}

/**
 * Clears the selected GPNVM bit.
 * @param pPrivate info about the bank
 * @param gpnvm GPNVM index.
 * @returns 0 if successful; otherwise returns an error code.
 */
static int FLASHD_ClrGPNVM(struct sam4_bank_private *pPrivate, unsigned gpnvm)
{
	int r;
	unsigned v;

	LOG_DEBUG("Here");
	if (pPrivate->bank_number != 0) {
		LOG_ERROR("GPNVM only works with Bank0");
		return ERROR_FAIL;
	}

	if (gpnvm >= pPrivate->pChip->details.n_gpnvms) {
		LOG_ERROR("Invalid GPNVM %d, max: %d, ignored",
			gpnvm, pPrivate->pChip->details.n_gpnvms);
		return ERROR_FAIL;
	}

	r = FLASHD_GetGPNVM(pPrivate, gpnvm, &v);
	if (r != ERROR_OK) {
		LOG_DEBUG("Failed: %d", r);
		return r;
	}
	r = EFC_PerformCommand(pPrivate, AT91C_EFC_FCMD_CFB, gpnvm, NULL);
	LOG_DEBUG("End: %d", r);
	return r;
}

/**
 * Sets the selected GPNVM bit.
 * @param pPrivate info about the bank
 * @param gpnvm GPNVM index.
 */
static int FLASHD_SetGPNVM(struct sam4_bank_private *pPrivate, unsigned gpnvm)
{
	int r;
	unsigned v;

	if (pPrivate->bank_number != 0) {
		LOG_ERROR("GPNVM only works with Bank0");
		return ERROR_FAIL;
	}

	if (gpnvm >= pPrivate->pChip->details.n_gpnvms) {
		LOG_ERROR("Invalid GPNVM %d, max: %d, ignored",
			gpnvm, pPrivate->pChip->details.n_gpnvms);
		return ERROR_FAIL;
	}

	r = FLASHD_GetGPNVM(pPrivate, gpnvm, &v);
	if (r != ERROR_OK)
		return r;
	if (v) {
		/* already set */
		r = ERROR_OK;
	} else {
		/* set it */
		r = EFC_PerformCommand(pPrivate, AT91C_EFC_FCMD_SFB, gpnvm, NULL);
	}
	return r;
}

/**
 * Returns a bit field (at most 64) of locked regions within a page.
 * @param pPrivate info about the bank
 * @param v where to store locked bits
 */
static int FLASHD_GetLockBits(struct sam4_bank_private *pPrivate, uint32_t *v)
{
	int r;
	LOG_DEBUG("Here");
	r = EFC_PerformCommand(pPrivate, AT91C_EFC_FCMD_GLB, 0, NULL);
	if (r == ERROR_OK)	{
		EFC_GetResult(pPrivate, v);
		EFC_GetResult(pPrivate, v);
		EFC_GetResult(pPrivate, v);
		r = EFC_GetResult(pPrivate, v);
	}
	LOG_DEBUG("End: %d", r);
	return r;
}

/**
 * Unlocks all the regions in the given address range.
 * @param pPrivate info about the bank
 * @param start_sector first sector to unlock
 * @param end_sector last (inclusive) to unlock
 */

static int FLASHD_Unlock(struct sam4_bank_private *pPrivate,
	unsigned start_sector,
	unsigned end_sector)
{
	int r;
	uint32_t status;
	uint32_t pg;
	uint32_t pages_per_sector;

	pages_per_sector = pPrivate->sector_size / pPrivate->page_size;

	/* Unlock all pages */
	while (start_sector <= end_sector) {
		pg = start_sector * pages_per_sector;

		r = EFC_PerformCommand(pPrivate, AT91C_EFC_FCMD_CLB, pg, &status);
		if (r != ERROR_OK)
			return r;
		start_sector++;
	}

	return ERROR_OK;
}

/**
 * Locks regions
 * @param pPrivate - info about the bank
 * @param start_sector - first sector to lock
 * @param end_sector   - last sector (inclusive) to lock
 */
static int FLASHD_Lock(struct sam4_bank_private *pPrivate,
	unsigned start_sector,
	unsigned end_sector)
{
	uint32_t status;
	uint32_t pg;
	uint32_t pages_per_sector;
	int r;

	pages_per_sector = pPrivate->sector_size / pPrivate->page_size;

	/* Lock all pages */
	while (start_sector <= end_sector) {
		pg = start_sector * pages_per_sector;

		r = EFC_PerformCommand(pPrivate, AT91C_EFC_FCMD_SLB, pg, &status);
		if (r != ERROR_OK)
			return r;
		start_sector++;
	}
	return ERROR_OK;
}

/****** END SAM4 CODE ********/

/* begin helpful debug code */
/* print the fieldname, the field value, in dec & hex, and return field value */
static uint32_t sam4_reg_fieldname(struct sam4_chip *pChip,
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
	"Cortex-M7",				/* 0 */
	"arm946es",					/* 1 */
	"arm7tdmi",					/* 2 */
	"Cortex-M3",				/* 3 */
	"arm920t",					/* 4 */
	"arm926ejs",				/* 5 */
	"Cortex-A5",				/* 6 */
	"Cortex-M4",				/* 7 */
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
	"160K bytes",				/*  8 */
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
	{ 0x3C, "ATSAM4E"                                                               },
	{ 0x40,  "AT91x40 Series"                                                       },
	{ 0x42,  "AT91x42 Series"                                                       },
	{ 0x43,  "SAMG51 Series"
	},
	{ 0x44,  "SAMG55 Series (49-pin WLCSP)"                                         },
	{ 0x45,  "SAMG55 Series (64-pin)"                                                        },
	{ 0x47,  "SAMG53 Series"
	},
	{ 0x55,  "AT91x55 Series"                                                       },
	{ 0x60,  "AT91SAM7Axx Series"                                           },
	{ 0x61,  "AT91SAM7AQxx Series"                                          },
	{ 0x63,  "AT91x63 Series"                                                       },
	{ 0x64,  "SAM4CxxC (100-pin version)"                                           },
	{ 0x66,  "SAM4CxxE (144-pin version)"                                           },
	{ 0x70,  "AT91SAM7Sxx Series"                                           },
	{ 0x71,  "AT91SAM7XCxx Series"                                          },
	{ 0x72,  "AT91SAM7SExx Series"                                          },
	{ 0x73,  "AT91SAM7Lxx Series"                                           },
	{ 0x75,  "AT91SAM7Xxx Series"                                           },
	{ 0x76,  "AT91SAM7SLxx Series"                                          },
	{ 0x80,  "ATSAM3UxC Series (100-pin version)"           },
	{ 0x81,  "ATSAM3UxE Series (144-pin version)"           },
	{ 0x83,  "ATSAM3A/SAM4A xC Series (100-pin version)"},
	{ 0x84,  "ATSAM3X/SAM4X xC Series (100-pin version)"},
	{ 0x85,  "ATSAM3X/SAM4X xE Series (144-pin version)"},
	{ 0x86,  "ATSAM3X/SAM4X xG Series (208/217-pin version)"	},
	{ 0x88,  "ATSAM3S/SAM4S xA Series (48-pin version)"	},
	{ 0x89,  "ATSAM3S/SAM4S xB Series (64-pin version)"	},
	{ 0x8A,  "ATSAM3S/SAM4S xC Series (100-pin version)"},
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

static void sam4_explain_ckgr_mor(struct sam4_chip *pChip)
{
	uint32_t v;
	uint32_t rcen;

	v = sam4_reg_fieldname(pChip, "MOSCXTEN", pChip->cfg.CKGR_MOR, 0, 1);
	LOG_USER("(main xtal enabled: %s)", _yes_or_no(v));
	v = sam4_reg_fieldname(pChip, "MOSCXTBY", pChip->cfg.CKGR_MOR, 1, 1);
	LOG_USER("(main osc bypass: %s)", _yes_or_no(v));
	rcen = sam4_reg_fieldname(pChip, "MOSCRCEN", pChip->cfg.CKGR_MOR, 3, 1);
	LOG_USER("(onchip RC-OSC enabled: %s)", _yes_or_no(rcen));
	v = sam4_reg_fieldname(pChip, "MOSCRCF", pChip->cfg.CKGR_MOR, 4, 3);
	LOG_USER("(onchip RC-OSC freq: %s)", _rc_freq[v]);

	pChip->cfg.rc_freq = 0;
	if (rcen) {
		switch (v) {
			default:
				pChip->cfg.rc_freq = 0;
				break;
			case 0:
				pChip->cfg.rc_freq = 4 * 1000 * 1000;
				break;
			case 1:
				pChip->cfg.rc_freq = 8 * 1000 * 1000;
				break;
			case 2:
				pChip->cfg.rc_freq = 12 * 1000 * 1000;
				break;
		}
	}

	v = sam4_reg_fieldname(pChip, "MOSCXTST", pChip->cfg.CKGR_MOR, 8, 8);
	LOG_USER("(startup clks, time= %f uSecs)",
		((float)(v * 1000000)) / ((float)(pChip->cfg.slow_freq)));
	v = sam4_reg_fieldname(pChip, "MOSCSEL", pChip->cfg.CKGR_MOR, 24, 1);
	LOG_USER("(mainosc source: %s)",
		v ? "external xtal" : "internal RC");

	v = sam4_reg_fieldname(pChip, "CFDEN", pChip->cfg.CKGR_MOR, 25, 1);
	LOG_USER("(clock failure enabled: %s)",
		_yes_or_no(v));
}

static void sam4_explain_chipid_cidr(struct sam4_chip *pChip)
{
	int x;
	uint32_t v;
	const char *cp;

	sam4_reg_fieldname(pChip, "Version", pChip->cfg.CHIPID_CIDR, 0, 5);
	LOG_USER_N("\n");

	v = sam4_reg_fieldname(pChip, "EPROC", pChip->cfg.CHIPID_CIDR, 5, 3);
	LOG_USER("%s", eproc_names[v]);

	v = sam4_reg_fieldname(pChip, "NVPSIZE", pChip->cfg.CHIPID_CIDR, 8, 4);
	LOG_USER("%s", nvpsize[v]);

	v = sam4_reg_fieldname(pChip, "NVPSIZE2", pChip->cfg.CHIPID_CIDR, 12, 4);
	LOG_USER("%s", nvpsize2[v]);

	v = sam4_reg_fieldname(pChip, "SRAMSIZE", pChip->cfg.CHIPID_CIDR, 16, 4);
	LOG_USER("%s", sramsize[v]);

	v = sam4_reg_fieldname(pChip, "ARCH", pChip->cfg.CHIPID_CIDR, 20, 8);
	cp = _unknown;
	for (x = 0; archnames[x].name; x++) {
		if (v == archnames[x].value) {
			cp = archnames[x].name;
			break;
		}
	}

	LOG_USER("%s", cp);

	v = sam4_reg_fieldname(pChip, "NVPTYP", pChip->cfg.CHIPID_CIDR, 28, 3);
	LOG_USER("%s", nvptype[v]);

	v = sam4_reg_fieldname(pChip, "EXTID", pChip->cfg.CHIPID_CIDR, 31, 1);
	LOG_USER("(exists: %s)", _yes_or_no(v));
}

static void sam4_explain_ckgr_mcfr(struct sam4_chip *pChip)
{
	uint32_t v;

	v = sam4_reg_fieldname(pChip, "MAINFRDY", pChip->cfg.CKGR_MCFR, 16, 1);
	LOG_USER("(main ready: %s)", _yes_or_no(v));

	v = sam4_reg_fieldname(pChip, "MAINF", pChip->cfg.CKGR_MCFR, 0, 16);

	v = (v * pChip->cfg.slow_freq) / 16;
	pChip->cfg.mainosc_freq = v;

	LOG_USER("(%3.03f Mhz (%" PRIu32 ".%03" PRIu32 "khz slowclk)",
		_tomhz(v),
		(uint32_t)(pChip->cfg.slow_freq / 1000),
		(uint32_t)(pChip->cfg.slow_freq % 1000));
}

static void sam4_explain_ckgr_plla(struct sam4_chip *pChip)
{
	uint32_t mula, diva;

	diva = sam4_reg_fieldname(pChip, "DIVA", pChip->cfg.CKGR_PLLAR, 0, 8);
	LOG_USER_N("\n");
	mula = sam4_reg_fieldname(pChip, "MULA", pChip->cfg.CKGR_PLLAR, 16, 11);
	LOG_USER_N("\n");
	pChip->cfg.plla_freq = 0;
	if (mula == 0)
		LOG_USER("\tPLLA Freq: (Disabled,mula = 0)");
	else if (diva == 0)
		LOG_USER("\tPLLA Freq: (Disabled,diva = 0)");
	else if (diva >= 1) {
		pChip->cfg.plla_freq = (pChip->cfg.mainosc_freq * (mula + 1) / diva);
		LOG_USER("\tPLLA Freq: %3.03f MHz",
			_tomhz(pChip->cfg.plla_freq));
	}
}

static void sam4_explain_mckr(struct sam4_chip *pChip)
{
	uint32_t css, pres, fin = 0;
	int pdiv = 0;
	const char *cp = NULL;

	css = sam4_reg_fieldname(pChip, "CSS", pChip->cfg.PMC_MCKR, 0, 2);
	switch (css & 3) {
		case 0:
			fin = pChip->cfg.slow_freq;
			cp = "slowclk";
			break;
		case 1:
			fin = pChip->cfg.mainosc_freq;
			cp  = "mainosc";
			break;
		case 2:
			fin = pChip->cfg.plla_freq;
			cp  = "plla";
			break;
		case 3:
			if (pChip->cfg.CKGR_UCKR & (1 << 16)) {
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
	pres = sam4_reg_fieldname(pChip, "PRES", pChip->cfg.PMC_MCKR, 4, 3);
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
	/* sam4 has a *SINGLE* clock - */
	/* other at91 series parts have divisors for these. */
	pChip->cfg.cpu_freq = fin;
	pChip->cfg.mclk_freq = fin;
	pChip->cfg.fclk_freq = fin;
	LOG_USER("\t\tResult CPU Freq: %3.03f",
		_tomhz(fin));
}

#if 0
static struct sam4_chip *target2sam4(struct target *pTarget)
{
	struct sam4_chip *pChip;

	if (pTarget == NULL)
		return NULL;

	pChip = all_sam4_chips;
	while (pChip) {
		if (pChip->target == pTarget)
			break;	/* return below */
		else
			pChip = pChip->next;
	}
	return pChip;
}
#endif

static uint32_t *sam4_get_reg_ptr(struct sam4_cfg *pCfg, const struct sam4_reg_list *pList)
{
	/* this function exists to help */
	/* keep funky offsetof() errors */
	/* and casting from causing bugs */

	/* By using prototypes - we can detect what would */
	/* be casting errors. */

	return (uint32_t *)(void *)(((char *)(pCfg)) + pList->struct_offset);
}


#define SAM4_ENTRY(NAME, FUNC)  { .address = SAM4_ ## NAME, .struct_offset = offsetof( \
						  struct sam4_cfg, \
						  NAME), # NAME, FUNC }
static const struct sam4_reg_list sam4_all_regs[] = {
	SAM4_ENTRY(CKGR_MOR, sam4_explain_ckgr_mor),
	SAM4_ENTRY(CKGR_MCFR, sam4_explain_ckgr_mcfr),
	SAM4_ENTRY(CKGR_PLLAR, sam4_explain_ckgr_plla),
	SAM4_ENTRY(CKGR_UCKR, NULL),
	SAM4_ENTRY(PMC_FSMR, NULL),
	SAM4_ENTRY(PMC_FSPR, NULL),
	SAM4_ENTRY(PMC_IMR, NULL),
	SAM4_ENTRY(PMC_MCKR, sam4_explain_mckr),
	SAM4_ENTRY(PMC_PCK0, NULL),
	SAM4_ENTRY(PMC_PCK1, NULL),
	SAM4_ENTRY(PMC_PCK2, NULL),
	SAM4_ENTRY(PMC_PCSR, NULL),
	SAM4_ENTRY(PMC_SCSR, NULL),
	SAM4_ENTRY(PMC_SR, NULL),
	SAM4_ENTRY(CHIPID_CIDR, sam4_explain_chipid_cidr),
	SAM4_ENTRY(CHIPID_EXID, NULL),
	/* TERMINATE THE LIST */
	{ .name = NULL }
};
#undef SAM4_ENTRY

static struct sam4_bank_private *get_sam4_bank_private(struct flash_bank *bank)
{
	return bank->driver_priv;
}

/**
 * Given a pointer to where it goes in the structure,
 * determine the register name, address from the all registers table.
 */
static const struct sam4_reg_list *sam4_GetReg(struct sam4_chip *pChip, uint32_t *goes_here)
{
	const struct sam4_reg_list *pReg;

	pReg = &(sam4_all_regs[0]);
	while (pReg->name) {
		uint32_t *pPossible;

		/* calculate where this one go.. */
		/* it is "possibly" this register. */

		pPossible = ((uint32_t *)(void *)(((char *)(&(pChip->cfg))) + pReg->struct_offset));

		/* well? Is it this register */
		if (pPossible == goes_here) {
			/* Jump for joy! */
			return pReg;
		}

		/* next... */
		pReg++;
	}
	/* This is *TOTAL*PANIC* - we are totally screwed. */
	LOG_ERROR("INVALID SAM4 REGISTER");
	return NULL;
}

static int sam4_ReadThisReg(struct sam4_chip *pChip, uint32_t *goes_here)
{
	const struct sam4_reg_list *pReg;
	int r;

	pReg = sam4_GetReg(pChip, goes_here);
	if (!pReg)
		return ERROR_FAIL;

	r = target_read_u32(pChip->target, pReg->address, goes_here);
	if (r != ERROR_OK) {
		LOG_ERROR("Cannot read SAM4 register: %s @ 0x%08x, Err: %d",
			pReg->name, (unsigned)(pReg->address), r);
	}
	return r;
}

static int sam4_ReadAllRegs(struct sam4_chip *pChip)
{
	int r;
	const struct sam4_reg_list *pReg;

	pReg = &(sam4_all_regs[0]);
	while (pReg->name) {
		r = sam4_ReadThisReg(pChip,
				sam4_get_reg_ptr(&(pChip->cfg), pReg));
		if (r != ERROR_OK) {
			LOG_ERROR("Cannot read SAM4 register: %s @ 0x%08x, Error: %d",
				pReg->name, ((unsigned)(pReg->address)), r);
			return r;
		}
		pReg++;
	}

	return ERROR_OK;
}

static int sam4_GetInfo(struct sam4_chip *pChip)
{
	const struct sam4_reg_list *pReg;
	uint32_t regval;
	int r;

	r = sam4_ReadAllRegs(pChip);
	if (r != ERROR_OK)
		return r;

	pReg = &(sam4_all_regs[0]);
	while (pReg->name) {
		/* display all regs */
		LOG_DEBUG("Start: %s", pReg->name);
		regval = *sam4_get_reg_ptr(&(pChip->cfg), pReg);
		LOG_USER("%*s: [0x%08" PRIx32 "] -> 0x%08" PRIx32,
			REG_NAME_WIDTH,
			pReg->name,
			pReg->address,
			regval);
		if (pReg->explain_func)
			(*(pReg->explain_func))(pChip);
		LOG_DEBUG("End: %s", pReg->name);
		pReg++;
	}
	LOG_USER("   rc-osc: %3.03f MHz", _tomhz(pChip->cfg.rc_freq));
	LOG_USER("  mainosc: %3.03f MHz", _tomhz(pChip->cfg.mainosc_freq));
	LOG_USER("     plla: %3.03f MHz", _tomhz(pChip->cfg.plla_freq));
	LOG_USER(" cpu-freq: %3.03f MHz", _tomhz(pChip->cfg.cpu_freq));
	LOG_USER("mclk-freq: %3.03f MHz", _tomhz(pChip->cfg.mclk_freq));

	LOG_USER(" UniqueId: 0x%08" PRIx32 " 0x%08" PRIx32 " 0x%08" PRIx32 " 0x%08"PRIx32,
		pChip->cfg.unique_id[0],
		pChip->cfg.unique_id[1],
		pChip->cfg.unique_id[2],
		pChip->cfg.unique_id[3]);

	return ERROR_OK;
}

static int sam4_protect_check(struct flash_bank *bank)
{
	int r;
	uint32_t v[4] = {0};
	unsigned x;
	struct sam4_bank_private *pPrivate;

	LOG_DEBUG("Begin");
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	pPrivate = get_sam4_bank_private(bank);
	if (!pPrivate) {
		LOG_ERROR("no private for this bank?");
		return ERROR_FAIL;
	}
	if (!(pPrivate->probed))
		return ERROR_FLASH_BANK_NOT_PROBED;

	r = FLASHD_GetLockBits(pPrivate, v);
	if (r != ERROR_OK) {
		LOG_DEBUG("Failed: %d", r);
		return r;
	}

	for (x = 0; x < pPrivate->nsectors; x++)
		bank->sectors[x].is_protected = (!!(v[x >> 5] & (1 << (x % 32))));
	LOG_DEBUG("Done");
	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(sam4_flash_bank_command)
{
	struct sam4_chip *pChip;

	pChip = all_sam4_chips;

	/* is this an existing chip? */
	while (pChip) {
		if (pChip->target == bank->target)
			break;
		pChip = pChip->next;
	}

	if (!pChip) {
		/* this is a *NEW* chip */
		pChip = calloc(1, sizeof(struct sam4_chip));
		if (!pChip) {
			LOG_ERROR("NO RAM!");
			return ERROR_FAIL;
		}
		pChip->target = bank->target;
		/* insert at head */
		pChip->next = all_sam4_chips;
		all_sam4_chips = pChip;
		pChip->target = bank->target;
		/* assumption is this runs at 32khz */
		pChip->cfg.slow_freq = 32768;
		pChip->probed = false;
	}

	switch (bank->base) {
		default:
			LOG_ERROR("Address 0x%08x invalid bank address (try 0x%08x"
				"[at91sam4s series] )",
				((unsigned int)(bank->base)),
				((unsigned int)(FLASH_BANK_BASE_S)));
			return ERROR_FAIL;

		/* at91sam4s series only has bank 0*/
		/* at91sam4sd series has the same address for bank 0 (FLASH_BANK0_BASE_SD)*/
		case FLASH_BANK_BASE_S:
		case FLASH_BANK_BASE_C:
			bank->driver_priv = &(pChip->details.bank[0]);
			bank->bank_number = 0;
			pChip->details.bank[0].pChip = pChip;
			pChip->details.bank[0].pBank = bank;
			break;

		/* Bank 1 of at91sam4sd/at91sam4c32 series */
		case FLASH_BANK1_BASE_1024K_SD:
		case FLASH_BANK1_BASE_2048K_SD:
		case FLASH_BANK1_BASE_C32:
			bank->driver_priv = &(pChip->details.bank[1]);
			bank->bank_number = 1;
			pChip->details.bank[1].pChip = pChip;
			pChip->details.bank[1].pBank = bank;
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
static void sam4_free_driver_priv(struct flash_bank *bank)
{
	struct sam4_chip *chip = all_sam4_chips;
	while (chip) {
		struct sam4_chip *next = chip->next;
		free(chip);
		chip = next;
	}
	all_sam4_chips = NULL;
}

static int sam4_GetDetails(struct sam4_bank_private *pPrivate)
{
	const struct sam4_chip_details *pDetails;
	struct sam4_chip *pChip;
	struct flash_bank *saved_banks[SAM4_MAX_FLASH_BANKS];
	unsigned x;

	LOG_DEBUG("Begin");
	pDetails = all_sam4_details;
	while (pDetails->name) {
		/* Compare cidr without version bits */
		if (pDetails->chipid_cidr == (pPrivate->pChip->cfg.CHIPID_CIDR & 0xFFFFFFE0))
			break;
		else
			pDetails++;
	}
	if (pDetails->name == NULL) {
		LOG_ERROR("SAM4 ChipID 0x%08x not found in table (perhaps you can ID this chip?)",
			(unsigned int)(pPrivate->pChip->cfg.CHIPID_CIDR));
		/* Help the victim, print details about the chip */
		LOG_INFO("SAM4 CHIPID_CIDR: 0x%08" PRIx32 " decodes as follows",
			pPrivate->pChip->cfg.CHIPID_CIDR);
		sam4_explain_chipid_cidr(pPrivate->pChip);
		return ERROR_FAIL;
	} else {
		LOG_DEBUG("SAM4 Found chip %s, CIDR 0x%08" PRIx32, pDetails->name, pDetails->chipid_cidr);
	}

	/* DANGER: THERE ARE DRAGONS HERE */

	/* get our pChip - it is going */
	/* to be over-written shortly */
	pChip = pPrivate->pChip;

	/* Note that, in reality: */
	/*  */
	/*     pPrivate = &(pChip->details.bank[0]) */
	/* or  pPrivate = &(pChip->details.bank[1]) */
	/*  */

	/* save the "bank" pointers */
	for (x = 0; x < SAM4_MAX_FLASH_BANKS; x++)
		saved_banks[x] = pChip->details.bank[x].pBank;

	/* Overwrite the "details" structure. */
	memcpy(&(pPrivate->pChip->details),
		pDetails,
		sizeof(pPrivate->pChip->details));

	/* now fix the ghosted pointers */
	for (x = 0; x < SAM4_MAX_FLASH_BANKS; x++) {
		pChip->details.bank[x].pChip = pChip;
		pChip->details.bank[x].pBank = saved_banks[x];
	}

	/* update the *BANK*SIZE* */

	LOG_DEBUG("End");
	return ERROR_OK;
}

static int sam4_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct sam4_bank_private *pPrivate;
	int k = bank->size / 1024;

	pPrivate = get_sam4_bank_private(bank);
	if (pPrivate == NULL) {
		buf[0] = '\0';
		return ERROR_FAIL;
	}

	snprintf(buf, buf_size,
		"%s bank %d: %d kB at " TARGET_ADDR_FMT,
		pPrivate->pChip->details.name,
		pPrivate->bank_number,
		k,
		bank->base);

	return ERROR_OK;
}

static int sam4_probe(struct flash_bank *bank)
{
	int r;
	struct sam4_bank_private *pPrivate;


	LOG_DEBUG("Begin: Bank: %u", bank->bank_number);
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	pPrivate = get_sam4_bank_private(bank);
	if (!pPrivate) {
		LOG_ERROR("Invalid/unknown bank number");
		return ERROR_FAIL;
	}

	r = sam4_ReadAllRegs(pPrivate->pChip);
	if (r != ERROR_OK)
		return r;

	LOG_DEBUG("Here");
	if (pPrivate->pChip->probed)
		r = sam4_GetInfo(pPrivate->pChip);
	else
		r = sam4_GetDetails(pPrivate);
	if (r != ERROR_OK)
		return r;

	/* update the flash bank size */
	for (unsigned int x = 0; x < SAM4_MAX_FLASH_BANKS; x++) {
		if (bank->base == pPrivate->pChip->details.bank[x].base_address) {
			bank->size = pPrivate->pChip->details.bank[x].size_bytes;
			LOG_DEBUG("SAM4 Set flash bank to " TARGET_ADDR_FMT " - "
					TARGET_ADDR_FMT ", idx %d", bank->base,
					bank->base + bank->size, x);
			break;
		}
	}

	if (bank->sectors == NULL) {
		bank->sectors = calloc(pPrivate->nsectors, (sizeof((bank->sectors)[0])));
		if (bank->sectors == NULL) {
			LOG_ERROR("No memory!");
			return ERROR_FAIL;
		}
		bank->num_sectors = pPrivate->nsectors;

		for (unsigned int x = 0; x < bank->num_sectors; x++) {
			bank->sectors[x].size = pPrivate->sector_size;
			bank->sectors[x].offset = x * (pPrivate->sector_size);
			/* mark as unknown */
			bank->sectors[x].is_erased = -1;
			bank->sectors[x].is_protected = -1;
		}
	}

	pPrivate->probed = true;

	r = sam4_protect_check(bank);
	if (r != ERROR_OK)
		return r;

	LOG_DEBUG("Bank = %d, nbanks = %d",
		pPrivate->bank_number, pPrivate->pChip->details.n_banks);
	if ((pPrivate->bank_number + 1) == pPrivate->pChip->details.n_banks) {
		/* read unique id, */
		/* it appears to be associated with the *last* flash bank. */
		FLASHD_ReadUniqueID(pPrivate);
	}

	return r;
}

static int sam4_auto_probe(struct flash_bank *bank)
{
	struct sam4_bank_private *pPrivate;

	pPrivate = get_sam4_bank_private(bank);
	if (pPrivate && pPrivate->probed)
		return ERROR_OK;

	return sam4_probe(bank);
}

static int sam4_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct sam4_bank_private *pPrivate;
	int r;
	int pageCount;
	/*16 pages equals 8KB - Same size as a lock region*/
	pageCount = 16;
	uint32_t status;

	LOG_DEBUG("Here");
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	r = sam4_auto_probe(bank);
	if (r != ERROR_OK) {
		LOG_DEBUG("Here,r=%d", r);
		return r;
	}

	pPrivate = get_sam4_bank_private(bank);
	if (!(pPrivate->probed))
		return ERROR_FLASH_BANK_NOT_PROBED;

	if ((first == 0) && ((last + 1) == pPrivate->nsectors)) {
		/* whole chip */
		LOG_DEBUG("Here");
		return FLASHD_EraseEntireBank(pPrivate);
	}
	LOG_INFO("sam4 does not auto-erase while programming (Erasing relevant sectors)");
	LOG_INFO("sam4 First: 0x%08x Last: 0x%08x", first, last);
	for (unsigned int i = first; i <= last; i++) {
		/*16 pages equals 8KB - Same size as a lock region*/
		r = FLASHD_ErasePages(pPrivate, (i * pageCount), pageCount, &status);
		LOG_INFO("Erasing sector: 0x%08x", i);
		if (r != ERROR_OK)
			LOG_ERROR("SAM4: Error performing Erase page @ lock region number %u",
				i);
		if (status & (1 << 2)) {
			LOG_ERROR("SAM4: Lock Region %u is locked", i);
			return ERROR_FAIL;
		}
		if (status & (1 << 1)) {
			LOG_ERROR("SAM4: Flash Command error @lock region %u", i);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int sam4_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct sam4_bank_private *pPrivate;
	int r;

	LOG_DEBUG("Here");
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	pPrivate = get_sam4_bank_private(bank);
	if (!(pPrivate->probed))
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (set)
		r = FLASHD_Lock(pPrivate, first, last);
	else
		r = FLASHD_Unlock(pPrivate, first, last);
	LOG_DEBUG("End: r=%d", r);

	return r;

}

static int sam4_page_read(struct sam4_bank_private *pPrivate, unsigned pagenum, uint8_t *buf)
{
	uint32_t adr;
	int r;

	adr = pagenum * pPrivate->page_size;
	adr = adr + pPrivate->base_address;

	r = target_read_memory(pPrivate->pChip->target,
			adr,
			4,					/* THIS*MUST*BE* in 32bit values */
			pPrivate->page_size / 4,
			buf);
	if (r != ERROR_OK)
		LOG_ERROR("SAM4: Flash program failed to read page phys address: 0x%08x",
			(unsigned int)(adr));
	return r;
}

static int sam4_set_wait(struct sam4_bank_private *pPrivate)
{
	uint32_t fmr;	/* EEFC Flash Mode Register */
	int r;

	/* Get flash mode register value */
	r = target_read_u32(pPrivate->pChip->target, pPrivate->controller_address, &fmr);
	if (r != ERROR_OK) {
		LOG_ERROR("Error Read failed: read flash mode register");
		return r;
	}

	/* Clear flash wait state field */
	fmr &= 0xfffff0ff;

	/* set FWS (flash wait states) field in the FMR (flash mode register) */
	fmr |= (pPrivate->flash_wait_states << 8);

	LOG_DEBUG("Flash Mode: 0x%08x", ((unsigned int)(fmr)));
	r = target_write_u32(pPrivate->pBank->target, pPrivate->controller_address, fmr);
	if (r != ERROR_OK)
		LOG_ERROR("Error Write failed: set flash mode register");

	return r;
}

static int sam4_page_write(struct sam4_bank_private *pPrivate, unsigned pagenum, const uint8_t *buf)
{
	uint32_t adr;
	uint32_t status;
	int r;

	adr = pagenum * pPrivate->page_size;
	adr = (adr + pPrivate->base_address);

	/* 1st sector 8kBytes - page 0 - 15*/
	/* 2nd sector 8kBytes - page 16 - 30*/
	/* 3rd sector 48kBytes - page 31 - 127*/
	LOG_DEBUG("Wr Page %u @ phys address: 0x%08x", pagenum, (unsigned int)(adr));
	r = target_write_memory(pPrivate->pChip->target,
			adr,
			4,					/* THIS*MUST*BE* in 32bit values */
			pPrivate->page_size / 4,
			buf);
	if (r != ERROR_OK) {
		LOG_ERROR("SAM4: Failed to write (buffer) page at phys address 0x%08x",
			(unsigned int)(adr));
		return r;
	}

	r = EFC_PerformCommand(pPrivate,
			/* send Erase & Write Page */
			AT91C_EFC_FCMD_WP,	/*AT91C_EFC_FCMD_EWP only works on first two 8kb sectors*/
			pagenum,
			&status);

	if (r != ERROR_OK)
		LOG_ERROR("SAM4: Error performing Write page @ phys address 0x%08x",
			(unsigned int)(adr));
	if (status & (1 << 2)) {
		LOG_ERROR("SAM4: Page @ Phys address 0x%08x is locked", (unsigned int)(adr));
		return ERROR_FAIL;
	}
	if (status & (1 << 1)) {
		LOG_ERROR("SAM4: Flash Command error @phys address 0x%08x", (unsigned int)(adr));
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int sam4_write(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	int n;
	unsigned page_cur;
	unsigned page_end;
	int r;
	unsigned page_offset;
	struct sam4_bank_private *pPrivate;
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

	pPrivate = get_sam4_bank_private(bank);
	if (!(pPrivate->probed)) {
		r = ERROR_FLASH_BANK_NOT_PROBED;
		goto done;
	}

	if ((offset + count) > pPrivate->size_bytes) {
		LOG_ERROR("Flash write error - past end of bank");
		LOG_ERROR(" offset: 0x%08x, count 0x%08x, BankEnd: 0x%08x",
			(unsigned int)(offset),
			(unsigned int)(count),
			(unsigned int)(pPrivate->size_bytes));
		r = ERROR_FAIL;
		goto done;
	}

	pagebuffer = malloc(pPrivate->page_size);
	if (!pagebuffer) {
		LOG_ERROR("No memory for %d Byte page buffer", (int)(pPrivate->page_size));
		r = ERROR_FAIL;
		goto done;
	}

	r = sam4_set_wait(pPrivate);
	if (r != ERROR_OK)
		goto done;

	/* what page do we start & end in? */
	page_cur = offset / pPrivate->page_size;
	page_end = (offset + count - 1) / pPrivate->page_size;

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
		r = sam4_page_read(pPrivate, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;

		page_offset = (offset & (pPrivate->page_size-1));
		memcpy(pagebuffer + page_offset,
			buffer,
			count);

		r = sam4_page_write(pPrivate, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;
		r = ERROR_OK;
		goto done;
	}

	/* non-aligned start */
	page_offset = offset & (pPrivate->page_size - 1);
	if (page_offset) {
		LOG_DEBUG("Not-Aligned start");
		/* read the partial */
		r = sam4_page_read(pPrivate, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;

		/* over-write with new data */
		n = (pPrivate->page_size - page_offset);
		memcpy(pagebuffer + page_offset,
			buffer,
			n);

		r = sam4_page_write(pPrivate, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;

		count  -= n;
		offset += n;
		buffer += n;
		page_cur++;
	}

	/* By checking that offset is correct here, we also
	fix a clang warning */
	assert(offset % pPrivate->page_size == 0);

	/* intermediate large pages */
	/* also - the final *terminal* */
	/* if that terminal page is a full page */
	LOG_DEBUG("Full Page Loop: cur=%d, end=%d, count = 0x%08x",
		(int)page_cur, (int)page_end, (unsigned int)(count));

	while ((page_cur < page_end) &&
			(count >= pPrivate->page_size)) {
		r = sam4_page_write(pPrivate, page_cur, buffer);
		if (r != ERROR_OK)
			goto done;
		count -= pPrivate->page_size;
		buffer += pPrivate->page_size;
		page_cur += 1;
	}

	/* terminal partial page? */
	if (count) {
		LOG_DEBUG("Terminal partial page, count = 0x%08x", (unsigned int)(count));
		/* we have a partial page */
		r = sam4_page_read(pPrivate, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;
					/* data goes at start */
		memcpy(pagebuffer, buffer, count);
		r = sam4_page_write(pPrivate, page_cur, pagebuffer);
		if (r != ERROR_OK)
			goto done;
	}
	LOG_DEBUG("Done!");
	r = ERROR_OK;
done:
	free(pagebuffer);
	return r;
}

COMMAND_HANDLER(sam4_handle_info_command)
{
	struct sam4_chip *pChip;
	pChip = get_current_sam4(CMD);
	if (!pChip)
		return ERROR_OK;

	unsigned x;
	int r;

	/* bank0 must exist before we can do anything */
	if (pChip->details.bank[0].pBank == NULL) {
		x = 0;
need_define:
		command_print(CMD,
			"Please define bank %d via command: flash bank %s ... ",
			x,
			at91sam4_flash.name);
		return ERROR_FAIL;
	}

	/* if bank 0 is not probed, then probe it */
	if (!(pChip->details.bank[0].probed)) {
		r = sam4_auto_probe(pChip->details.bank[0].pBank);
		if (r != ERROR_OK)
			return ERROR_FAIL;
	}
	/* above guarantees the "chip details" structure is valid */
	/* and thus, bank private areas are valid */
	/* and we have a SAM4 chip, what a concept! */

	/* auto-probe other banks, 0 done above */
	for (x = 1; x < SAM4_MAX_FLASH_BANKS; x++) {
		/* skip banks not present */
		if (!(pChip->details.bank[x].present))
			continue;

		if (pChip->details.bank[x].pBank == NULL)
			goto need_define;

		if (pChip->details.bank[x].probed)
			continue;

		r = sam4_auto_probe(pChip->details.bank[x].pBank);
		if (r != ERROR_OK)
			return r;
	}

	r = sam4_GetInfo(pChip);
	if (r != ERROR_OK) {
		LOG_DEBUG("Sam4Info, Failed %d", r);
		return r;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(sam4_handle_gpnvm_command)
{
	unsigned x, v;
	int r, who;
	struct sam4_chip *pChip;

	pChip = get_current_sam4(CMD);
	if (!pChip)
		return ERROR_OK;

	if (pChip->target->state != TARGET_HALTED) {
		LOG_ERROR("sam4 - target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (pChip->details.bank[0].pBank == NULL) {
		command_print(CMD, "Bank0 must be defined first via: flash bank %s ...",
			at91sam4_flash.name);
		return ERROR_FAIL;
	}
	if (!pChip->details.bank[0].probed) {
		r = sam4_auto_probe(pChip->details.bank[0].pBank);
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
			if ((0 == strcmp(CMD_ARGV[0], "show")) && (0 == strcmp(CMD_ARGV[1], "all")))
				who = -1;
			else {
				uint32_t v32;
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], v32);
				who = v32;
			}
			break;
	}

	if (0 == strcmp("show", CMD_ARGV[0])) {
		if (who == -1) {
showall:
			r = ERROR_OK;
			for (x = 0; x < pChip->details.n_gpnvms; x++) {
				r = FLASHD_GetGPNVM(&(pChip->details.bank[0]), x, &v);
				if (r != ERROR_OK)
					break;
				command_print(CMD, "sam4-gpnvm%u: %u", x, v);
			}
			return r;
		}
		if ((who >= 0) && (((unsigned)(who)) < pChip->details.n_gpnvms)) {
			r = FLASHD_GetGPNVM(&(pChip->details.bank[0]), who, &v);
			if (r == ERROR_OK)
				command_print(CMD, "sam4-gpnvm%u: %u", who, v);
			return r;
		} else {
			command_print(CMD, "sam4-gpnvm invalid GPNVM: %u", who);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (who == -1) {
		command_print(CMD, "Missing GPNVM number");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (0 == strcmp("set", CMD_ARGV[0]))
		r = FLASHD_SetGPNVM(&(pChip->details.bank[0]), who);
	else if ((0 == strcmp("clr", CMD_ARGV[0])) ||
		 (0 == strcmp("clear", CMD_ARGV[0])))			/* quietly accept both */
		r = FLASHD_ClrGPNVM(&(pChip->details.bank[0]), who);
	else {
		command_print(CMD, "Unknown command: %s", CMD_ARGV[0]);
		r = ERROR_COMMAND_SYNTAX_ERROR;
	}
	return r;
}

COMMAND_HANDLER(sam4_handle_slowclk_command)
{
	struct sam4_chip *pChip;

	pChip = get_current_sam4(CMD);
	if (!pChip)
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
			pChip->cfg.slow_freq = v;
			break;
		}
		default:
			/* error */
			command_print(CMD, "Too many parameters");
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	command_print(CMD, "Slowclk freq: %d.%03dkhz",
		(int)(pChip->cfg.slow_freq / 1000),
		(int)(pChip->cfg.slow_freq % 1000));
	return ERROR_OK;
}

static const struct command_registration at91sam4_exec_command_handlers[] = {
	{
		.name = "gpnvm",
		.handler = sam4_handle_gpnvm_command,
		.mode = COMMAND_EXEC,
		.usage = "[('clr'|'set'|'show') bitnum]",
		.help = "Without arguments, shows all bits in the gpnvm "
			"register.  Otherwise, clears, sets, or shows one "
			"General Purpose Non-Volatile Memory (gpnvm) bit.",
	},
	{
		.name = "info",
		.handler = sam4_handle_info_command,
		.mode = COMMAND_EXEC,
		.help = "Print information about the current at91sam4 chip "
			"and its flash configuration.",
		.usage = "",
	},
	{
		.name = "slowclk",
		.handler = sam4_handle_slowclk_command,
		.mode = COMMAND_EXEC,
		.usage = "[clock_hz]",
		.help = "Display or set the slowclock frequency "
			"(default 32768 Hz).",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration at91sam4_command_handlers[] = {
	{
		.name = "at91sam4",
		.mode = COMMAND_ANY,
		.help = "at91sam4 flash command group",
		.usage = "",
		.chain = at91sam4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver at91sam4_flash = {
	.name = "at91sam4",
	.commands = at91sam4_command_handlers,
	.flash_bank_command = sam4_flash_bank_command,
	.erase = sam4_erase,
	.protect = sam4_protect,
	.write = sam4_write,
	.read = default_flash_read,
	.probe = sam4_probe,
	.auto_probe = sam4_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = sam4_protect_check,
	.info = sam4_info,
	.free_driver_priv = sam4_free_driver_priv,
};
