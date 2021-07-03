/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Gheorghe Guran (atlas)                          *
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

/***************************************************************************
*
* New flash setup command:
*
* flash bank <driver> <base_addr> <size> <chip_width> <bus_width> <target_id>
*	[<chip_type> <banks>
*	 <sectors_per_bank> <pages_per_sector>
*	 <page_size> <num_nvmbits>
*	 <ext_freq_khz>]
*
*   <ext_freq_khz> - MUST be used if clock is from external source,
*                    CAN be used if main oscillator frequency is known (recommended)
* Examples:
* ==== RECOMMENDED (covers clock speed) ============
*  flash bank at91sam7 0x00100000 0 0 4 $_TARGETNAME AT91SAM7XC256 1 16 64 256 3 25000
*			(if auto-detect fails; provides clock spec)
*  flash bank at91sam7 0 0 0 0 $_TARGETNAME 0 0 0 0 0 0 25000
*			(auto-detect everything except the clock)
* ==== NOT RECOMMENDED !!! (clock speed is not configured) ====
*  flash bank at91sam7 0x00100000 0 0 4 $_TARGETNAME AT91SAM7XC256 1 16 64 256 3 0
*			(if auto-detect fails)
*  flash bank at91sam7 0 0 0 0 $_TARGETNAME
*			(old style, auto-detect everything)
****************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>

/* AT91SAM7 control registers */
#define DBGU_CIDR                       0xFFFFF240
#define CKGR_MCFR                       0xFFFFFC24
#define CKGR_MOR                        0xFFFFFC20
#define CKGR_MCFR_MAINRDY       0x10000
#define CKGR_PLLR                       0xFFFFFC2c
#define CKGR_PLLR_DIV           0xff
#define CKGR_PLLR_MUL           0x07ff0000
#define PMC_MCKR                        0xFFFFFC30
#define PMC_MCKR_CSS            0x03
#define PMC_MCKR_PRES           0x1c

/* Flash Controller Commands */
#define WP              0x01
#define SLB             0x02
#define WPL             0x03
#define CLB             0x04
#define EA              0x08
#define SGPB    0x0B
#define CGPB    0x0D
#define SSB             0x0F

/* MC_FSR bit definitions */
#define MC_FSR_FRDY                     1
#define MC_FSR_EOL                      2

/* AT91SAM7 constants */
#define RC_FREQ                         32000

/* Flash timing modes */
#define FMR_TIMING_NONE         0
#define FMR_TIMING_NVBITS       1
#define FMR_TIMING_FLASH        2

/* Flash size constants */
#define FLASH_SIZE_8KB          1
#define FLASH_SIZE_16KB         2
#define FLASH_SIZE_32KB         3
#define FLASH_SIZE_64KB         5
#define FLASH_SIZE_128KB        7
#define FLASH_SIZE_256KB        9
#define FLASH_SIZE_512KB        10
#define FLASH_SIZE_1024KB       12
#define FLASH_SIZE_2048KB       14

static int at91sam7_protect_check(struct flash_bank *bank);
static int at91sam7_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
		uint32_t count);

static uint32_t at91sam7_get_flash_status(struct target *target, int bank_number);
static void at91sam7_set_flash_mode(struct flash_bank *bank, int mode);
static uint32_t at91sam7_wait_status_busy(struct flash_bank *bank, uint32_t waitbits, int timeout);
static int at91sam7_flash_command(struct flash_bank *bank, uint8_t cmd, uint16_t pagen);

static const uint32_t mc_fmr[4] = { 0xFFFFFF60, 0xFFFFFF70, 0xFFFFFF80, 0xFFFFFF90 };
static const uint32_t mc_fcr[4] = { 0xFFFFFF64, 0xFFFFFF74, 0xFFFFFF84, 0xFFFFFF94 };
static const uint32_t mc_fsr[4] = { 0xFFFFFF68, 0xFFFFFF78, 0xFFFFFF88, 0xFFFFFF98 };

static const char *eproc[8] = {
	"Unknown", "ARM946-E", "ARM7TDMI", "Unknown", "ARM920T", "ARM926EJ-S", "Unknown", "Unknown"
};

struct at91sam7_flash_bank {
	/* chip id register */
	uint32_t cidr;
	uint16_t cidr_ext;
	uint16_t cidr_nvptyp;
	uint16_t cidr_arch;
	uint16_t cidr_sramsiz;
	uint16_t cidr_nvpsiz;
	uint16_t cidr_nvpsiz2;
	uint16_t cidr_eproc;
	uint16_t cidr_version;
	const char *target_name;

	/* flash auto-detection */
	uint8_t flash_autodetection;

	/* flash geometry */
	uint16_t pages_per_sector;
	uint16_t pagesize;
	uint16_t pages_in_lockregion;

	/* nv memory bits */
	uint16_t num_lockbits_on;
	uint16_t lockbits;
	uint16_t num_nvmbits;
	uint16_t num_nvmbits_on;
	uint16_t nvmbits;
	uint8_t securitybit;

	/* 0: not init
	 * 1: fmcn for nvbits (1uS)
	 * 2: fmcn for flash (1.5uS) */
	uint8_t flashmode;

	/* main clock status */
	uint8_t mck_valid;
	uint32_t mck_freq;

	/* external clock frequency */
	uint32_t ext_freq;

};

#if 0
static long SRAMSIZ[16] = {
	-1,
	0x0400,		/*  1K */
	0x0800,		/*  2K */
	-1,
	0x1c000,	/* 112K */
	0x1000,		/*   4K */
	0x14000,	/*  80K */
	0x28000,	/* 160K */
	0x2000,		/*   8K */
	0x4000,		/*  16K */
	0x8000,		/*  32K */
	0x10000,	/*  64K */
	0x20000,	/* 128K */
	0x40000,	/* 256K */
	0x18000,	/*  96K */
	0x80000,	/* 512K */
};
#endif

static uint32_t at91sam7_get_flash_status(struct target *target, int bank_number)
{
	uint32_t fsr;
	target_read_u32(target, mc_fsr[bank_number], &fsr);

	return fsr;
}

/* Read clock configuration and set at91sam7_info->mck_freq */
static void at91sam7_read_clock_info(struct flash_bank *bank)
{
	struct at91sam7_flash_bank *at91sam7_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t mckr, mcfr, pllr, mor;
	unsigned long tmp = 0, mainfreq;

	/* Read Clock Generator Main Oscillator Register */
	target_read_u32(target, CKGR_MOR, &mor);
	/* Read Clock Generator Main Clock Frequency Register */
	target_read_u32(target, CKGR_MCFR, &mcfr);
	/* Read Master Clock Register*/
	target_read_u32(target, PMC_MCKR, &mckr);
	/* Read Clock Generator PLL Register  */
	target_read_u32(target, CKGR_PLLR, &pllr);

	at91sam7_info->mck_valid = 0;
	at91sam7_info->mck_freq = 0;
	switch (mckr & PMC_MCKR_CSS) {
		case 0:			/* Slow Clock */
			at91sam7_info->mck_valid = 1;
			tmp = RC_FREQ;
			break;

		case 1:			/* Main Clock */
			if ((mcfr & CKGR_MCFR_MAINRDY) &&
			(at91sam7_info->ext_freq == 0)) {
				at91sam7_info->mck_valid = 1;
				tmp = RC_FREQ / 16ul * (mcfr & 0xffff);
			} else if (at91sam7_info->ext_freq != 0) {
				at91sam7_info->mck_valid = 1;
				tmp = at91sam7_info->ext_freq;
			}
			break;

		case 2:			/* Reserved */
			break;

		case 3:			/* PLL Clock */
			if ((mcfr & CKGR_MCFR_MAINRDY) &&
			(at91sam7_info->ext_freq == 0)) {
				target_read_u32(target, CKGR_PLLR, &pllr);
				if (!(pllr & CKGR_PLLR_DIV))
					break;	/* 0 Hz */
				at91sam7_info->mck_valid = 1;
				mainfreq = RC_FREQ / 16ul * (mcfr & 0xffff);
				/* Integer arithmetic should have sufficient precision
				 * as long as PLL is properly configured. */
				tmp = mainfreq / (pllr & CKGR_PLLR_DIV)*
						(((pllr & CKGR_PLLR_MUL) >> 16) + 1);
			} else if ((at91sam7_info->ext_freq != 0) &&
					((pllr&CKGR_PLLR_DIV) != 0)) {
				at91sam7_info->mck_valid = 1;
				tmp = at91sam7_info->ext_freq / (pllr&CKGR_PLLR_DIV)*
						(((pllr & CKGR_PLLR_MUL) >> 16) + 1);
			}
			break;
	}

	/* Prescaler adjust */
	if ((((mckr & PMC_MCKR_PRES) >> 2) == 7) || (tmp == 0)) {
		at91sam7_info->mck_valid = 0;
		at91sam7_info->mck_freq = 0;
	} else if (((mckr & PMC_MCKR_PRES) >> 2) != 0)
		at91sam7_info->mck_freq = tmp >> ((mckr & PMC_MCKR_PRES) >> 2);
	else
		at91sam7_info->mck_freq = tmp;
}

/* Setup the timing registers for nvbits or normal flash */
static void at91sam7_set_flash_mode(struct flash_bank *bank, int mode)
{
	uint32_t fmr, fmcn = 0, fws = 0;
	struct at91sam7_flash_bank *at91sam7_info = bank->driver_priv;
	struct target *target = bank->target;

	if (mode && (mode != at91sam7_info->flashmode)) {
		/* Always round up (ceil) */
		if (mode == FMR_TIMING_NVBITS) {
			if (at91sam7_info->cidr_arch == 0x60) {
				/* AT91SAM7A3 uses master clocks in 100 ns */
				fmcn = (at91sam7_info->mck_freq/10000000ul) + 1;
			} else {
				/* master clocks in 1uS for ARCH 0x7 types */
				fmcn = (at91sam7_info->mck_freq/1000000ul) + 1;
			}
		} else if (mode == FMR_TIMING_FLASH) {
			/* main clocks in 1.5uS */
			fmcn = (at91sam7_info->mck_freq/1000000ul)+
				(at91sam7_info->mck_freq/2000000ul) + 1;
		}

		/* hard overclocking */
		if (fmcn > 0xFF)
			fmcn = 0xFF;

		/* Only allow fmcn = 0 if clock period is > 30 us = 33kHz. */
		if (at91sam7_info->mck_freq <= 33333ul)
			fmcn = 0;
		/* Only allow fws = 0 if clock frequency is < 30 MHz. */
		if (at91sam7_info->mck_freq > 30000000ul)
			fws = 1;

		LOG_DEBUG("fmcn[%i]: %i", bank->bank_number, (int)(fmcn));
		fmr = fmcn << 16 | fws << 8;
		target_write_u32(target, mc_fmr[bank->bank_number], fmr);
	}

	at91sam7_info->flashmode = mode;
}

static uint32_t at91sam7_wait_status_busy(struct flash_bank *bank, uint32_t waitbits, int timeout)
{
	uint32_t status;

	while ((!((status = at91sam7_get_flash_status(bank->target,
			bank->bank_number)) & waitbits)) && (timeout-- > 0)) {
		LOG_DEBUG("status[%i]: 0x%" PRIx32 "", (int)bank->bank_number, status);
		alive_sleep(1);
	}

	LOG_DEBUG("status[%i]: 0x%" PRIx32 "", bank->bank_number, status);

	if (status & 0x0C) {
		LOG_ERROR("status register: 0x%" PRIx32 "", status);
		if (status & 0x4)
			LOG_ERROR("Lock Error Bit Detected, Operation Abort");
		if (status & 0x8)
			LOG_ERROR("Invalid command and/or bad keyword, Operation Abort");
		if (status & 0x10)
			LOG_ERROR("Security Bit Set, Operation Abort");
	}

	return status;
}

/* Send one command to the AT91SAM flash controller */
static int at91sam7_flash_command(struct flash_bank *bank, uint8_t cmd, uint16_t pagen)
{
	uint32_t fcr;
	struct at91sam7_flash_bank *at91sam7_info = bank->driver_priv;
	struct target *target = bank->target;

	fcr = (0x5A << 24) | ((pagen&0x3FF) << 8) | cmd;
	target_write_u32(target, mc_fcr[bank->bank_number], fcr);
	LOG_DEBUG("Flash command: 0x%" PRIx32 ", flash bank: %i, page number: %u",
		fcr,
		bank->bank_number + 1,
		pagen);

	if ((at91sam7_info->cidr_arch == 0x60) && ((cmd == SLB) | (cmd == CLB))) {
		/* Lock bit manipulation on AT91SAM7A3 waits for FC_FSR bit 1, EOL */
		if (at91sam7_wait_status_busy(bank, MC_FSR_EOL, 10)&0x0C)
			return ERROR_FLASH_OPERATION_FAILED;
		return ERROR_OK;
	}

	if (at91sam7_wait_status_busy(bank, MC_FSR_FRDY, 10)&0x0C)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

/* Read device id register, main clock frequency register and fill in driver info structure */
static int at91sam7_read_part_info(struct flash_bank *bank)
{
	struct at91sam7_flash_bank *at91sam7_info;
	struct target *target = bank->target;

	uint16_t bnk, sec;
	uint16_t arch;
	uint32_t cidr;
	uint8_t banks_num = 0;
	uint16_t num_nvmbits = 0;
	uint16_t sectors_num = 0;
	uint16_t pages_per_sector = 0;
	uint16_t page_size = 0;
	uint32_t ext_freq;
	uint32_t bank_size;
	uint32_t base_address = 0;
	char *target_name_t = "Unknown";

	at91sam7_info = bank->driver_priv;

	if (at91sam7_info->cidr != 0) {
		/* flash already configured, update clock and check for protected sectors */
		for (struct flash_bank *t_bank = bank; t_bank; t_bank = t_bank->next) {
			if (t_bank->target != target)
				continue;
			/* re-calculate master clock frequency */
			at91sam7_read_clock_info(t_bank);

			/* no timing */
			at91sam7_set_flash_mode(t_bank, FMR_TIMING_NONE);

			/* check protect state */
			at91sam7_protect_check(t_bank);
		}

		return ERROR_OK;
	}

	/* Read and parse chip identification register */
	target_read_u32(target, DBGU_CIDR, &cidr);
	if (cidr == 0) {
		LOG_WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (at91sam7_info->flash_autodetection == 0) {
		/* banks and sectors are already created, based on data from input file */
		for (struct flash_bank *t_bank = bank; t_bank; t_bank = t_bank->next) {
			if (t_bank->target != target)
				continue;

			at91sam7_info = t_bank->driver_priv;

			at91sam7_info->cidr = cidr;
			at91sam7_info->cidr_ext = (cidr >> 31)&0x0001;
			at91sam7_info->cidr_nvptyp = (cidr >> 28)&0x0007;
			at91sam7_info->cidr_arch = (cidr >> 20)&0x00FF;
			at91sam7_info->cidr_sramsiz = (cidr >> 16)&0x000F;
			at91sam7_info->cidr_nvpsiz2 = (cidr >> 12)&0x000F;
			at91sam7_info->cidr_nvpsiz = (cidr >> 8)&0x000F;
			at91sam7_info->cidr_eproc = (cidr >> 5)&0x0007;
			at91sam7_info->cidr_version = cidr&0x001F;

			/* calculate master clock frequency */
			at91sam7_read_clock_info(t_bank);

			/* no timing */
			at91sam7_set_flash_mode(t_bank, FMR_TIMING_NONE);

			/* check protect state */
			at91sam7_protect_check(t_bank);
		}

		return ERROR_OK;
	}

	arch = (cidr >> 20)&0x00FF;

	/* check flash size */
	switch ((cidr >> 8)&0x000F) {
		case FLASH_SIZE_8KB:
			break;

		case FLASH_SIZE_16KB:
			banks_num = 1;
			sectors_num = 8;
			pages_per_sector = 32;
			page_size  = 64;
			base_address = 0x00100000;
			if (arch == 0x70) {
				num_nvmbits = 2;
				target_name_t = "AT91SAM7S161/16";
			}
			break;

		case FLASH_SIZE_32KB:
			banks_num = 1;
			sectors_num = 8;
			pages_per_sector = 32;
			page_size  = 128;
			base_address = 0x00100000;
			if (arch == 0x70) {
				num_nvmbits = 2;
				target_name_t = "AT91SAM7S321/32";
			}
			if (arch == 0x72) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7SE32";
			}
			break;

		case FLASH_SIZE_64KB:
			banks_num = 1;
			sectors_num = 16;
			pages_per_sector = 32;
			page_size  = 128;
			base_address = 0x00100000;
			if (arch == 0x70) {
				num_nvmbits = 2;
				target_name_t = "AT91SAM7S64";
			}
			break;

		case FLASH_SIZE_128KB:
			banks_num = 1;
			sectors_num = 8;
			pages_per_sector = 64;
			page_size  = 256;
			base_address = 0x00100000;
			if (arch == 0x70) {
				num_nvmbits = 2;
				target_name_t = "AT91SAM7S128";
			}
			if (arch == 0x71) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7XC128";
			}
			if (arch == 0x72) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7SE128";
			}
			if (arch == 0x75) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7X128";
			}
			break;

		case FLASH_SIZE_256KB:
			banks_num = 1;
			sectors_num = 16;
			pages_per_sector = 64;
			page_size  = 256;
			base_address = 0x00100000;
			if (arch == 0x60) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7A3";
			}
			if (arch == 0x70) {
				num_nvmbits = 2;
				target_name_t = "AT91SAM7S256";
			}
			if (arch == 0x71) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7XC256";
			}
			if (arch == 0x72) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7SE256";
			}
			if (arch == 0x75) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7X256";
			}
			break;

		case FLASH_SIZE_512KB:
			banks_num = 2;
			sectors_num = 16;
			pages_per_sector = 64;
			page_size  = 256;
			base_address = 0x00100000;
			if (arch == 0x70) {
				num_nvmbits = 2;
				target_name_t = "AT91SAM7S512";
			}
			if (arch == 0x71) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7XC512";
			}
			if (arch == 0x72) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7SE512";
			}
			if (arch == 0x75) {
				num_nvmbits = 3;
				target_name_t = "AT91SAM7X512";
			}
			break;

		case FLASH_SIZE_1024KB:
			break;

		case FLASH_SIZE_2048KB:
			break;
	}

	if (strcmp(target_name_t, "Unknown") == 0) {
		LOG_ERROR(
			"Target autodetection failed! Please specify target parameters in configuration file");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	ext_freq = at91sam7_info->ext_freq;

	/* calculate bank size  */
	bank_size = sectors_num * pages_per_sector * page_size;

	for (bnk = 0; bnk < banks_num; bnk++) {
		struct flash_bank *t_bank = bank;
		if (bnk > 0) {
			if (!t_bank->next) {
				/* create a new flash bank element */
				struct flash_bank *fb = malloc(sizeof(struct flash_bank));
				fb->target = target;
				fb->driver = bank->driver;
				fb->driver_priv = malloc(sizeof(struct at91sam7_flash_bank));
				fb->name = "sam7_probed";
				fb->next = NULL;

				/* link created bank in 'flash_banks' list */
				t_bank->next = fb;
			}
			t_bank = t_bank->next;
		}

		t_bank->bank_number = bnk;
		t_bank->base = base_address + bnk * bank_size;
		t_bank->size = bank_size;
		t_bank->chip_width = 0;
		t_bank->bus_width = 4;
		t_bank->num_sectors = sectors_num;

		/* allocate sectors */
		t_bank->sectors = malloc(sectors_num * sizeof(struct flash_sector));
		for (sec = 0; sec < sectors_num; sec++) {
			t_bank->sectors[sec].offset = sec * pages_per_sector * page_size;
			t_bank->sectors[sec].size = pages_per_sector * page_size;
			t_bank->sectors[sec].is_erased = -1;
			t_bank->sectors[sec].is_protected = -1;
		}

		at91sam7_info = t_bank->driver_priv;

		at91sam7_info->cidr = cidr;
		at91sam7_info->cidr_ext = (cidr >> 31)&0x0001;
		at91sam7_info->cidr_nvptyp = (cidr >> 28)&0x0007;
		at91sam7_info->cidr_arch = (cidr >> 20)&0x00FF;
		at91sam7_info->cidr_sramsiz = (cidr >> 16)&0x000F;
		at91sam7_info->cidr_nvpsiz2 = (cidr >> 12)&0x000F;
		at91sam7_info->cidr_nvpsiz = (cidr >> 8)&0x000F;
		at91sam7_info->cidr_eproc = (cidr >> 5)&0x0007;
		at91sam7_info->cidr_version = cidr&0x001F;

		at91sam7_info->target_name  = target_name_t;
		at91sam7_info->flashmode = 0;
		at91sam7_info->ext_freq = ext_freq;
		at91sam7_info->num_nvmbits = num_nvmbits;
		at91sam7_info->num_nvmbits_on = 0;
		at91sam7_info->pagesize = page_size;
		at91sam7_info->pages_per_sector = pages_per_sector;

		/* calculate master clock frequency */
		at91sam7_read_clock_info(t_bank);

		/* no timing */
		at91sam7_set_flash_mode(t_bank, FMR_TIMING_NONE);

		/* check protect state */
		at91sam7_protect_check(t_bank);
	}

	LOG_DEBUG("nvptyp: 0x%3.3x, arch: 0x%4.4x",
		at91sam7_info->cidr_nvptyp,
		at91sam7_info->cidr_arch);

	return ERROR_OK;
}

static int at91sam7_erase_check(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_FLASH);

	return default_flash_blank_check(bank);
}

static int at91sam7_protect_check(struct flash_bank *bank)
{
	uint8_t lock_pos, gpnvm_pos;
	uint32_t status;

	struct at91sam7_flash_bank *at91sam7_info = bank->driver_priv;

	if (at91sam7_info->cidr == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	status = at91sam7_get_flash_status(bank->target, bank->bank_number);
	at91sam7_info->lockbits = (status >> 16);

	at91sam7_info->num_lockbits_on = 0;
	for (lock_pos = 0; lock_pos < bank->num_sectors; lock_pos++) {
		if (((status >> (16 + lock_pos))&(0x0001)) == 1) {
			at91sam7_info->num_lockbits_on++;
			bank->sectors[lock_pos].is_protected = 1;
		} else
			bank->sectors[lock_pos].is_protected = 0;
	}

	/* GPNVM and SECURITY bits apply only for MC_FSR of EFC0 */
	status = at91sam7_get_flash_status(bank->target, 0);

	at91sam7_info->securitybit = (status >> 4)&0x01;
	at91sam7_info->nvmbits = (status >> 8)&0xFF;

	at91sam7_info->num_nvmbits_on = 0;
	for (gpnvm_pos = 0; gpnvm_pos < at91sam7_info->num_nvmbits; gpnvm_pos++) {
		if (((status >> (8 + gpnvm_pos))&(0x01)) == 1)
			at91sam7_info->num_nvmbits_on++;
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(at91sam7_flash_bank_command)
{
	struct flash_bank *t_bank = bank;
	struct at91sam7_flash_bank *at91sam7_info;
	struct target *target = t_bank->target;

	uint32_t base_address;
	uint32_t bank_size;
	uint32_t ext_freq = 0;

	unsigned int chip_width;
	unsigned int bus_width;
	unsigned int banks_num;
	unsigned int num_sectors;

	uint16_t pages_per_sector;
	uint16_t page_size;
	uint16_t num_nvmbits;

	at91sam7_info = malloc(sizeof(struct at91sam7_flash_bank));
	t_bank->driver_priv = at91sam7_info;

	/* part wasn't probed for info yet */
	at91sam7_info->cidr = 0;
	at91sam7_info->flashmode = 0;
	at91sam7_info->ext_freq = 0;
	at91sam7_info->flash_autodetection = 0;

	if (CMD_ARGC < 13) {
		at91sam7_info->flash_autodetection = 1;
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], base_address);

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[3], chip_width);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[4], bus_width);

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[8], banks_num);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[9], num_sectors);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[10], pages_per_sector);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[11], page_size);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[12], num_nvmbits);

	if (CMD_ARGC == 14) {
		unsigned long freq;
		COMMAND_PARSE_NUMBER(ulong, CMD_ARGV[13], freq);
		ext_freq = freq * 1000;
		at91sam7_info->ext_freq = ext_freq;
	}

	if ((bus_width == 0) || (banks_num == 0) || (num_sectors == 0) ||
			(pages_per_sector == 0) || (page_size == 0) || (num_nvmbits == 0)) {
		at91sam7_info->flash_autodetection = 1;
		return ERROR_OK;
	}

	/* calculate bank size  */
	bank_size = num_sectors * pages_per_sector * page_size;

	for (unsigned int bnk = 0; bnk < banks_num; bnk++) {
		if (bnk > 0) {
			if (!t_bank->next) {
				/* create a new bank element */
				struct flash_bank *fb = malloc(sizeof(struct flash_bank));
				fb->target = target;
				fb->driver = bank->driver;
				fb->driver_priv = malloc(sizeof(struct at91sam7_flash_bank));
				fb->name = "sam7_probed";
				fb->next = NULL;

				/* link created bank in 'flash_banks' list */
				t_bank->next = fb;
			}
			t_bank = t_bank->next;
		}

		t_bank->bank_number = bnk;
		t_bank->base = base_address + bnk * bank_size;
		t_bank->size = bank_size;
		t_bank->chip_width = chip_width;
		t_bank->bus_width = bus_width;
		t_bank->num_sectors = num_sectors;

		/* allocate sectors */
		t_bank->sectors = malloc(num_sectors * sizeof(struct flash_sector));
		for (unsigned int sec = 0; sec < num_sectors; sec++) {
			t_bank->sectors[sec].offset = sec * pages_per_sector * page_size;
			t_bank->sectors[sec].size = pages_per_sector * page_size;
			t_bank->sectors[sec].is_erased = -1;
			t_bank->sectors[sec].is_protected = -1;
		}

		at91sam7_info = t_bank->driver_priv;

		at91sam7_info->target_name = strdup(CMD_ARGV[7]);
		at91sam7_info->flashmode = 0;
		at91sam7_info->ext_freq  = ext_freq;
		at91sam7_info->num_nvmbits = num_nvmbits;
		at91sam7_info->num_nvmbits_on = 0;
		at91sam7_info->pagesize = page_size;
		at91sam7_info->pages_per_sector = pages_per_sector;
	}

	return ERROR_OK;
}

static int at91sam7_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct at91sam7_flash_bank *at91sam7_info = bank->driver_priv;
	uint32_t nbytes, pos;
	uint8_t *buffer;
	uint8_t erase_all;

	if (at91sam7_info->cidr == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;

	erase_all = 0;
	if ((first == 0) && (last == (bank->num_sectors-1)))
		erase_all = 1;

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_FLASH);

	if (erase_all) {
		if (at91sam7_flash_command(bank, EA, 0) != ERROR_OK)
			return ERROR_FLASH_OPERATION_FAILED;
	} else {
		/* allocate and clean buffer  */
		nbytes = (last - first + 1) * bank->sectors[first].size;
		buffer = malloc(nbytes * sizeof(uint8_t));
		for (pos = 0; pos < nbytes; pos++)
			buffer[pos] = 0xFF;

		if (at91sam7_write(bank, buffer, bank->sectors[first].offset, nbytes) != ERROR_OK) {
			free(buffer);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		free(buffer);
	}

	/* mark erased sectors */
	for (unsigned int sec = first; sec <= last; sec++)
		bank->sectors[sec].is_erased = 1;

	return ERROR_OK;
}

static int at91sam7_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	uint32_t cmd;
	uint32_t pagen;

	struct at91sam7_flash_bank *at91sam7_info = bank->driver_priv;

	if (at91sam7_info->cidr == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((last < first) || (last >= bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_NVBITS);

	for (unsigned int sector = first; sector <= last; sector++) {
		if (set)
			cmd = SLB;
		else
			cmd = CLB;

		/* if we lock a page from one sector then entire sector will be locked, also,
		 * if we unlock a page from a locked sector, entire sector will be unlocked   */
		pagen = sector * at91sam7_info->pages_per_sector;

		if (at91sam7_flash_command(bank, cmd, pagen) != ERROR_OK)
			return ERROR_FLASH_OPERATION_FAILED;
	}

	at91sam7_protect_check(bank);

	return ERROR_OK;
}

static int at91sam7_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;
	struct at91sam7_flash_bank *at91sam7_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t dst_min_alignment, wcount, bytes_remaining = count;
	uint32_t first_page, last_page, pagen, buffer_pos;

	if (at91sam7_info->cidr == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	dst_min_alignment = at91sam7_info->pagesize;

	if (offset % dst_min_alignment) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required alignment 0x%" PRIx32 "",
			offset,
			dst_min_alignment);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (at91sam7_info->cidr_arch == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	first_page = offset/dst_min_alignment;
	last_page = DIV_ROUND_UP(offset + count, dst_min_alignment);

	LOG_DEBUG("first_page: %i, last_page: %i, count %i",
		(int)first_page,
		(int)last_page,
		(int)count);

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_FLASH);

	for (pagen = first_page; pagen < last_page; pagen++) {
		if (bytes_remaining < dst_min_alignment)
			count = bytes_remaining;
		else
			count = dst_min_alignment;
		bytes_remaining -= count;

		/* Write one block to the PageWriteBuffer */
		buffer_pos = (pagen-first_page)*dst_min_alignment;
		wcount = DIV_ROUND_UP(count, 4);
		retval = target_write_memory(target, bank->base + pagen*dst_min_alignment, 4,
				wcount, buffer + buffer_pos);
		if (retval != ERROR_OK)
			return retval;

		/* Send Write Page command to Flash Controller */
		if (at91sam7_flash_command(bank, WP, pagen) != ERROR_OK)
			return ERROR_FLASH_OPERATION_FAILED;
		LOG_DEBUG("Write flash bank:%u page number:%" PRIu32, bank->bank_number, pagen);
	}

	return ERROR_OK;
}

static int at91sam7_probe(struct flash_bank *bank)
{
	/* we can't probe on an at91sam7
	 * if this is an at91sam7, it has the configured flash */
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = at91sam7_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int get_at91sam7_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct at91sam7_flash_bank *at91sam7_info = bank->driver_priv;

	if (at91sam7_info->cidr == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	command_print_sameline(cmd, "\n at91sam7 driver information: Chip is %s\n",
			at91sam7_info->target_name);

	command_print_sameline(cmd,
			" Cidr: 0x%8.8" PRIx32 " | Arch: 0x%4.4x | Eproc: %s | Version: 0x%3.3x | "
			"Flashsize: 0x%8.8" PRIx32 "\n",
			at91sam7_info->cidr,
			at91sam7_info->cidr_arch,
			eproc[at91sam7_info->cidr_eproc],
			at91sam7_info->cidr_version,
			bank->size);

	command_print_sameline(cmd,
			" Master clock (estimated): %u kHz | External clock: %u kHz\n",
			(unsigned)(at91sam7_info->mck_freq / 1000),
			(unsigned)(at91sam7_info->ext_freq / 1000));

	command_print_sameline(cmd,
			" Pagesize: %i bytes | Lockbits(%u): %i 0x%4.4x | Pages in lock region: %i\n",
			at91sam7_info->pagesize,
			bank->num_sectors,
			at91sam7_info->num_lockbits_on,
			at91sam7_info->lockbits,
			at91sam7_info->pages_per_sector * at91sam7_info->num_lockbits_on);

	command_print_sameline(cmd, " Securitybit: %i | Nvmbits(%i): %i 0x%1.1x\n",
		at91sam7_info->securitybit, at91sam7_info->num_nvmbits,
		at91sam7_info->num_nvmbits_on, at91sam7_info->nvmbits);

	return ERROR_OK;
}

/*
* On AT91SAM7S: When the gpnvm bits are set with
* > at91sam7 gpnvm bitnr set
* the changes are not visible in the flash controller status register MC_FSR
* until the processor has been reset.
* On the Olimex board this requires a power cycle.
* Note that the AT91SAM7S has the following errata (doc6175.pdf sec 14.1.3):
*   The maximum number of write/erase cycles for Non volatile Memory bits is 100. this includes
*   Lock Bits (LOCKx), General Purpose NVM bits (GPNVMx) and the Security Bit.
*/
COMMAND_HANDLER(at91sam7_handle_gpnvm_command)
{
	struct flash_bank *bank;
	int bit;
	uint8_t flashcmd;
	uint32_t status;
	struct at91sam7_flash_bank *at91sam7_info;
	int retval;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bank = get_flash_bank_by_num_noprobe(0);
	if (!bank)
		return ERROR_FLASH_BANK_INVALID;
	if (strcmp(bank->driver->name, "at91sam7")) {
		command_print(CMD, "not an at91sam7 flash bank '%s'", CMD_ARGV[0]);
		return ERROR_FLASH_BANK_INVALID;
	}
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("target has to be halted to perform flash operation");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (strcmp(CMD_ARGV[1], "set") == 0)
		flashcmd = SGPB;
	else if (strcmp(CMD_ARGV[1], "clear") == 0)
		flashcmd = CGPB;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	at91sam7_info = bank->driver_priv;
	if (at91sam7_info->cidr == 0) {
		retval = at91sam7_read_part_info(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], bit);
	if ((bit < 0) || (bit >= at91sam7_info->num_nvmbits)) {
		command_print(CMD,
			"gpnvm bit '#%s' is out of bounds for target %s",
			CMD_ARGV[0],
			at91sam7_info->target_name);
		return ERROR_OK;
	}

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_NVBITS);

	if (at91sam7_flash_command(bank, flashcmd, bit) != ERROR_OK)
		return ERROR_FLASH_OPERATION_FAILED;

	/* GPNVM and SECURITY bits apply only for MC_FSR of EFC0 */
	status = at91sam7_get_flash_status(bank->target, 0);
	LOG_DEBUG("at91sam7_handle_gpnvm_command: cmd 0x%x, value %d, status 0x%" PRIx32,
		flashcmd,
		bit,
		status);

	/* check protect state */
	at91sam7_protect_check(bank);

	return ERROR_OK;
}

static const struct command_registration at91sam7_exec_command_handlers[] = {
	{
		.name = "gpnvm",
		.handler = at91sam7_handle_gpnvm_command,
		.mode = COMMAND_EXEC,
		.help = "set or clear one General Purpose Non-Volatile Memory "
			"(gpnvm) bit",
		.usage = "bitnum ('set'|'clear')",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration at91sam7_command_handlers[] = {
	{
		.name = "at91sam7",
		.mode = COMMAND_ANY,
		.help = "at91sam7 flash command group",
		.usage = "",
		.chain = at91sam7_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver at91sam7_flash = {
	.name = "at91sam7",
	.usage = "gpnvm <bit> <set | clear>",
	.commands = at91sam7_command_handlers,
	.flash_bank_command = at91sam7_flash_bank_command,
	.erase = at91sam7_erase,
	.protect = at91sam7_protect,
	.write = at91sam7_write,
	.read = default_flash_read,
	.probe = at91sam7_probe,
	.auto_probe = at91sam7_probe,
	.erase_check = at91sam7_erase_check,
	.protect_check = at91sam7_protect_check,
	.info = get_at91sam7_info,
};
