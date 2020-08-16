// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include "imp.h"
#include <target/algorithm.h>
#include <target/mips32.h>
#include <target/mips_m4k.h>

/* In PIC32MZ EC devices, the Flash page size is 16 KB (4096 IW) and the row
 * size is 2 KB (512 IW).
 */

#define PIC32MZ_MANUF_ID		0x029

/* pic32mz memory locations */

#define PIC32MZ_PHYS_RAM		0x00000000
#define PIC32MZ_PHYS_PGM_FLASH		0x1D000000
#define PIC32MZ_PHYS_PERIPHERALS	0x1F800000
#define PIC32MZ_PHYS_BOOT_FLASH		0x1FC00000
#define PIC32MZ_PHYS_LOWER_BOOT_ALIAS	0x1FC00000
#define PIC32MZ_PHYS_UPPER_BOOT_ALIAS	0x1FC20000
#define PIC32MZ_PHYS_BOOT_FLASH_END	0x1FC74000
#define PIC32MZ_PAGE_NUMBYTES		(16*1024)
#define PIC32MZ_BOOT_BLOCK_NUMBYTES     (80*1024)
#define PIC32MZ_BOOT_BLOCK_NUMPAGES     (PIC32MZ_BOOT_BLOCK_NUMBYTES/PIC32MZ_PAGE_NUMBYTES)


/*
 * Translate Virtual and Physical addresses.
 * Note: These macros only work for KSEG0/KSEG1 addresses.
 */

#define VIRT2PHYS(v)			((v) & 0x1FFFFFFF)

/* pic32mz configuration register locations */

#define PIC32MZ_DEVCP0			0xBFC4FFDC

#define PIC32MZ_NVMBWP_LBWP0_OFFSET	8
#define PIC32MZ_NVMBWP_UBWP0_OFFSET	0

#define PIC32MZ_NVMBWP_LBWPULOCK	0x00008000
#define PIC32MZ_NVMBWP_UBWPULOCK	0x00000080

#define PIC32MZ_NVMPWP_PWPULOCK		0x80000000
#define PIC32MZ_NVMPWP_PWP		0x00FFFFFF

/* pic32mz flash controller register locations */

#define PIC32MZ_NVMCON			0xBF800600
#define PIC32MZ_NVMCONCLR		0xBF800604
#define PIC32MZ_NVMCONSET		0xBF800608
#define PIC32MZ_NVMCONINV		0xBF80060C
#define NVMCON_NVMWR			BIT(15)
#define NVMCON_NVMWREN			BIT(14)
#define NVMCON_NVMERR			BIT(13)
#define NVMCON_LVDERR			BIT(12)
#define NVMCON_OP_PFM_ERASE		0x7
#define NVMCON_OP_PAGE_ERASE		0x4
#define NVMCON_OP_ROW_PROG		0x3
#define NVMCON_OP_WORD_PROG		0x1
#define NVMCON_OP_QUAD_WORD_PROG	0x2
#define NVMCON_OP_NOP			0x0

#define PIC32MZ_NVMKEY			0xBF800610
#define PIC32MZ_NVMADDR			0xBF800620
#define PIC32MZ_NVMADDRCLR		0xBF800624
#define PIC32MZ_NVMADDRSET		0xBF800628
#define PIC32MZ_NVMADDRINV		0xBF80062C
#define PIC32MZ_NVMDATA0		0xBF800630
#define PIC32MZ_NVMDATA1		0xBF800640
#define PIC32MZ_NVMDATA2		0xBF800650
#define PIC32MZ_NVMDATA3		0xBF800660
#define PIC32MZ_NVMSRCADDR		0xBF800670
#define PIC32MZ_DEVCFG3			0xBFC0FFC0
#define PIC32MZ_NVMPWP			0xBF800680
#define PIC32MZ_NVMBWP			0xBF800690
#define PIC32MZ_NVMBWPCLR		0xBF800694

#define PIC32MZ_CFGCON			0xBF800000
#define CFGCON_ECC			0x30

/* flash unlock keys */

#define NVMKEY1				0xAA996655
#define NVMKEY2				0x556699AA

struct pic32mz_flash_bank {
	int probed;
	int dev_type;		/* Default 0. 1 for Pic32mz1XX/2XX variant */
};

/*
 * DEVID values as per PIC32MZ Flash Programming Specification Rev J
 */

static const struct pic32mz_devs_s {
	uint32_t devid;
	const char *name;
	uint32_t kb_program_flash;
} pic32mz_devs[] = {
	/* PIC32MZ EMBEDDED CONNECTIVITY (EC) FAMILY DEVICE IDS */
	{0x05103053, "1024ECG064", 1024},
	{0x05108053, "1024ECH064", 1024},
	{0x05130053, "1024ECM064", 1024},
	{0x05104053, "2048ECG064", 2048},
	{0x05109053, "2048ECH064", 2048},
	{0x05131053, "2048ECM064", 2048},
	{0x0510D053, "1024ECG100", 1024},
	{0x05112053, "1024ECH100", 1024},
	{0x0513A053, "1024ECM100", 1024},
	{0x0510E053, "2048ECG100", 2048},
	{0x05113053, "2048ECH100", 2048},
	{0x0513B053, "2048ECM100", 2048},
	{0x05117053, "1024ECG124", 1024},
	{0x0511C053, "1024ECH124", 1024},
	{0x05144053, "1024ECM124", 1024},
	{0x05118053, "2048ECG124", 2048},
	{0x0511D053, "2048ECH124", 2048},
	{0x05145053, "2048ECM124", 2048},
	{0x05121053, "1024ECG144", 1024},
	{0x05126053, "1024ECH144", 1024},
	{0x0514E053, "1024ECM144", 1024},
	{0x05122053, "2048ECG144", 2048},
	{0x05127053, "2048ECH144", 2048},
	{0x0514F053, "2048ECM144", 2048},
	/* PIC32MZ EMBEDDED CONNECTIVITY WITH FPU (EF) FAMILY DEVICE IDS */
	{0x07201053, "0512EFE064", 512},
	{0x07206053, "0512EFF064", 512},
	{0x0722E053, "0512EFK064", 512},
	{0x07202053, "1024EFE064", 1024},
	{0x07207053, "1024EFF064", 1024},
	{0x0722F053, "1024EFK064", 1024},
	{0x07203053, "1024EFG064", 1024},
	{0x07208053, "1024EFH064", 1024},
	{0x07230053, "1024EFM064", 1024},
	{0x07204053, "2048EFG064", 2048},
	{0x07209053, "2048EFH064", 2048},
	{0x07231053, "2048EFM064", 2048},
	{0x0720B053, "0512EFE100", 512},
	{0x07210053, "0512EFF100", 512},
	{0x07238053, "0512EFK100", 512},
	{0x0720C053, "1024EFE100", 1024},
	{0x07211053, "1024EFF100", 1024},
	{0x07239053, "1024EFK100", 1024},
	{0x0720D053, "1024EFG100", 1024},
	{0x07212053, "1024EFH100", 1024},
	{0x0723A053, "1024EFM100", 1024},
	{0x0720E053, "2048EFG100", 2048},
	{0x07213053, "2048EFH100", 2048},
	{0x0723B053, "2048EFM100", 2048},
	{0x07215053, "0512EFE124", 512},
	{0x0721A053, "0512EFF124", 512},
	{0x07242053, "0512EFK124", 512},
	{0x07216053, "1024EFE124", 1024},
	{0x0721B053, "1024EFF124", 1024},
	{0x07243053, "1024EFK124", 1024},
	{0x07217053, "1024EFG124", 1024},
	{0x0721C053, "1024EFH124", 1024},
	{0x07244053, "1024EFM124", 1024},
	{0x07218053, "2048EFG124", 2048},
	{0x0721D053, "2048EFH124", 2048},
	{0x07245053, "2048EFM124", 2048},
	{0x0721F053, "0512EFE144", 512},
	{0x07224053, "0512EFF144", 512},
	{0x0724C053, "0512EFK144", 512},
	{0x07220053, "1024EFE144", 1024},
	{0x07225053, "1024EFF144", 1024},
	{0x0724D053, "1024EFK144", 1024},
	{0x07221053, "1024EFG144", 1024},
	{0x07226053, "1024EFH144", 1024},
	{0x0724E053, "1024EFM144", 1024},
	{0x07222053, "2048EFG144", 2048},
	{0x07227053, "2048EFH144", 2048},
	{0x0724F053, "2048EFM144", 2048},
	/* PIC32MZ GRAPHICS (DA) FAMILY DEVICE IDS */
	{0x05F0C053, "1025DAA169", 1024},
	{0x05F0D053, "1025DAB169", 1024},
	{0x05F0F053, "1064DAA169", 1024},
	{0x05F10053, "1064DAB169", 1024},
	{0x05F15053, "2025DAA169", 2048},
	{0x05F16053, "2025DAB169", 2048},
	{0x05F18053, "2064DAA169", 2048},
	{0x05F19053, "2064DAB169", 2048},
	{0x05F42053, "1025DAG169", 1024},
	{0x05F43053, "1025DAH169", 1024},
	{0x05F45053, "1064DAG169", 1024},
	{0x05F46053, "1064DAH169", 1024},
	{0x05F4B053, "2025DAG169", 2048},
	{0x05F4C053, "2025DAH169", 2048},
	{0x05F4E053, "2064DAG169", 2048},
	{0x05F4F053, "2064DAH169", 2048},
	{0x05F78053, "1025DAA176", 1024},
	{0x05F79053, "1025DAB176", 1024},
	{0x05F7B053, "1064DAA176", 1024},
	{0x05F7C053, "1064DAB176", 1024},
	{0x05F81053, "2025DAA176", 2048},
	{0x05F82053, "2025DAB176", 2048},
	{0x05F84053, "2064DAA176", 2048},
	{0x05F85053, "2064DAB176", 2048},
	{0x05FAE053, "1025DAG176", 1024},
	{0x05FAF053, "1025DAH176", 1024},
	{0x05FB1053, "1064DAG176", 1024},
	{0x05FB2053, "1064DAH176", 1024},
	{0x05FB7053, "2025DAG176", 2048},
	{0x05FB8053, "2025DAH176", 2048},
	{0x05FBA053, "2064DAG176", 2048},
	{0x05FBB053, "2064DAH176", 2048},
	{0x05F5D053, "1025DAA288", 1024},
	{0x05F5E053, "1025DAB288", 1024},
	{0x05F60053, "1064DAA288", 1024},
	{0x05F61053, "1064DAB288", 1024},
	{0x05F66053, "2025DAA288", 2048},
	{0x05F67053, "2025DAB288", 2048},
	{0x05F69053, "2064DAA288", 2048},
	{0x05F6A053, "2064DAB288", 2048},
	{0x00000000, NULL, 0}
};

static const struct pic32mz_devs_s *pic32mz_lookup_device(uint32_t device_id)
{
	int i;

	for (i = 0; pic32mz_devs[i].name != NULL; i++) {
		if (pic32mz_devs[i].devid == (device_id & 0x0fffffff)) {
			return &pic32mz_devs[i];
		}
	}
	return NULL;
}

/* flash bank pic32mz <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(pic32mz_flash_bank_command)
{
	struct pic32mz_flash_bank *pic32mz_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pic32mz_info = malloc(sizeof(struct pic32mz_flash_bank));
	bank->driver_priv = pic32mz_info;

	pic32mz_info->probed = 0;
	pic32mz_info->dev_type = 0;

	return ERROR_OK;
}

static uint32_t pic32mz_get_flash_status(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;

	target_read_u32(target, PIC32MZ_NVMCON, &status);

	return status;
}

static uint32_t pic32mz_wait_status_busy(struct flash_bank *bank, int timeout)
{
	uint32_t status;

	/* wait for busy to clear */
	while (((status = pic32mz_get_flash_status(bank)) & NVMCON_NVMWR)
	       && (timeout-- > 0)) {
		LOG_DEBUG("status: 0x%" PRIx32, status);
		alive_sleep(1);
	}
	if (timeout <= 0)
		LOG_DEBUG("timeout: status: 0x%" PRIx32, status);

	return status;
}

static int pic32mz_nvm_exec(struct flash_bank *bank, uint32_t op,
			    uint32_t timeout)
{
	struct target *target = bank->target;
	uint32_t status;

	target_write_u32(target, PIC32MZ_NVMCON, NVMCON_NVMWREN | op);

	/* unlock flash registers */
	/* write of 0 might not have been required for PIC32MX,
	 * but it is suggested for PIC32MZ */
	target_write_u32(target, PIC32MZ_NVMKEY, 0);
	target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY1);
	target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY2);

	/* start operation */
	target_write_u32(target, PIC32MZ_NVMCONSET, NVMCON_NVMWR);

	status = pic32mz_wait_status_busy(bank, timeout);

	/* lock flash registers */
	target_write_u32(target, PIC32MZ_NVMCONCLR, NVMCON_NVMWREN);

	return status;
}

static int pic32mz_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t cp0_address;
	uint32_t nvmpwp_address;
	uint32_t nvmbwp_address;
	uint32_t devcp0;
	uint32_t nvmpwp;
	uint32_t nvmbwp;
	int s;
	int num_pages;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	cp0_address = PIC32MZ_DEVCP0;
	nvmpwp_address = PIC32MZ_NVMPWP;
	nvmbwp_address = PIC32MZ_NVMBWP;

	target_read_u32(target, cp0_address, &devcp0);



	if ((devcp0 & (1 << 28)) == 0) { /* code protect bit */
		/* all pages protected */
		for (s = 0; s < bank->num_sectors; s++)
			bank->sectors[s].is_protected = 1;
	} else if (VIRT2PHYS(bank->base) == PIC32MZ_PHYS_LOWER_BOOT_ALIAS) {
		/* Boot flash write protection is divided into pages and is
		 * enabled by the LBWPx bits in the NVMBWP register.*/
		target_read_u32(target, nvmbwp_address, &nvmbwp);
		for (s = 0; s < bank->num_sectors && s < PIC32MZ_BOOT_BLOCK_NUMPAGES; s++) {
			int bit = (nvmbwp >> (PIC32MZ_NVMBWP_LBWP0_OFFSET+s)) & 0x1;
			bank->sectors[s].is_protected = (bit != 0);
		}
	} else if (VIRT2PHYS(bank->base) == PIC32MZ_PHYS_UPPER_BOOT_ALIAS) {
		/* Boot Flash write protection is divided into pages and is enabled by the UBWPx bits in the NVMBWP register.*/
		target_read_u32(target, nvmbwp_address, &nvmbwp);
		for (s = 0; s < bank->num_sectors && s < PIC32MZ_BOOT_BLOCK_NUMPAGES; s++) {
			int bit = (nvmbwp >> (PIC32MZ_NVMBWP_UBWP0_OFFSET+s)) & 0x1;
			bank->sectors[s].is_protected = (bit != 0);
		}
	} else {
		/* pgm flash */
		target_read_u32(target, nvmpwp_address, &nvmpwp);
		nvmpwp &= 0x00FFFFFF;
		num_pages = (nvmpwp/PIC32MZ_PAGE_NUMBYTES)+1;
		for (s = 0; s < bank->num_sectors; s++) {
			// protected if nvmpwp is not zero, and if the page index is less than or equal to the page index of nvmpwp
			bank->sectors[s].is_protected = (nvmpwp != 0x0) && (s < num_pages);
		}
	}

	return ERROR_OK;
}

static int pic32mz_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;
	uint32_t status;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1))
		&& (VIRT2PHYS(bank->base) == PIC32MZ_PHYS_PGM_FLASH)) {
		/* this will only erase the Program Flash (PFM), not the Boot Flash (BFM)
		 * we need to use the MTAP to perform a full erase */
		LOG_DEBUG("Erasing entire program flash");
		status = pic32mz_nvm_exec(bank, NVMCON_OP_PFM_ERASE, 50);
		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		return ERROR_OK;
	}

	for (i = first; i <= last; i++) {
		target_write_u32(target, PIC32MZ_NVMADDR, VIRT2PHYS(bank->base + bank->sectors[i].offset));

		status = pic32mz_nvm_exec(bank, NVMCON_OP_PAGE_ERASE, 10);

		if (status & NVMCON_NVMERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & NVMCON_LVDERR)
			return ERROR_FLASH_OPERATION_FAILED;
		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int pic32mz_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct target *target = bank->target;
	int sector;
	uint32_t nvmbwp;
	uint32_t nvmpwp;
	uint32_t nvmpwp_23_0;
	int num_pages;

	/* the calling function(s) validate that first and last are valid sector
	 * indices, no need to validate here */

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (VIRT2PHYS(bank->base) == PIC32MZ_PHYS_LOWER_BOOT_ALIAS) {
		/* Boot flash write protection is divided into pages and is
		 * enabled by the LBWPx bits in the NVMBWP register.*/
		target_read_u32(target, PIC32MZ_NVMBWP, &nvmbwp);

		if ((nvmbwp & PIC32MZ_NVMBWP_LBWPULOCK) == 0) {
			LOG_ERROR("Boot flash write protect registers are locked; unlocking requires device reset");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		for (sector = first; sector <= last; sector++) {
			if (set) {
				nvmbwp |= (1 << (PIC32MZ_NVMBWP_LBWP0_OFFSET + sector));
			} else {
				nvmbwp &= ~(1 << (PIC32MZ_NVMBWP_LBWP0_OFFSET + sector));
			}
		}

		/* changing the write protect bits requires unlocking flash
		 * registers */
		target_write_u32(target, PIC32MZ_NVMKEY, 0);
		target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY1);
		target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY2);
		/* write back nvmbwp */
		target_write_u32(target, PIC32MZ_NVMBWP, nvmbwp);
	} else if (VIRT2PHYS(bank->base) == PIC32MZ_PHYS_UPPER_BOOT_ALIAS) {
		/* Boot Flash write protection is divided into pages and is
		 * enabled by the UBWPx bits in the NVMBWP register.*/
		target_read_u32(target, PIC32MZ_NVMBWP, &nvmbwp);
		if ((nvmbwp & PIC32MZ_NVMBWP_UBWPULOCK) == 0) {
			LOG_ERROR("Boot flash write protect registers are locked; unlocking requires device reset");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		for (sector = first; sector <= last; sector++) {
			if (set) {
				nvmbwp |= (1 << (PIC32MZ_NVMBWP_UBWP0_OFFSET+sector));
			} else {
				nvmbwp &= ~(1 << (PIC32MZ_NVMBWP_UBWP0_OFFSET+sector));
			}
		}
		/* changing the write protect bits requires unlocking flash
		 * registers */
		target_write_u32(target, PIC32MZ_NVMKEY, 0);
		target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY1);
		target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY2);
		/* write back nvmbwp */
		target_write_u32(target, PIC32MZ_NVMBWP, nvmbwp);
	} else {
		/* pgm flash */
		target_read_u32(target, PIC32MZ_NVMPWP, &nvmpwp);
		if ((nvmpwp & PIC32MZ_NVMPWP_PWPULOCK) == 0) {
			// can't unprotect the page
			LOG_ERROR("Program flash write protect register is locked; unlocking requires device reset");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		nvmpwp_23_0 = nvmpwp & PIC32MZ_NVMPWP_PWP;
		if (nvmpwp_23_0 == 0)
			num_pages = 0;
		else
			num_pages = nvmpwp_23_0/PIC32MZ_PAGE_NUMBYTES) + 1;

		if (set) {
			if (first > num_pages) {
				LOG_ERROR("protected space must be contiguous below unprotected space");
				return ERROR_FLASH_SECTOR_INVALID;
			}
			/* need 4 offset just because 0 is special sentinel
			 * value and must be avoided */
			nvmpwp = (nvmpwp & PIC32MZ_NVMPWP_PWPULOCK)
				| ((last * PIC32MZ_PAGE_NUMBYTES) + 4);
		} else {
			if (last < num_pages-1) {
				LOG_ERROR("protected space must be contiguous below unprotected space");
				return ERROR_FLASH_SECTOR_INVALID;
			}
			if (first == 0) {
				nvmpwp = (nvmpwp & PIC32MZ_NVMPWP_PWPULOCK);
			} else {
				/* need 4 offset just because 0 is special
				 * sentinel value and must be avoided */
				nvmpwp = (nvmpwp & PIC32MZ_NVMPWP_PWPULOCK)
					| (((first-1) * PIC32MZ_PAGE_NUMBYTES) + 4);
			}
		}
		/* changing the write protect bits requires unlocking flash registers */
		target_write_u32(target, PIC32MZ_NVMKEY, 0);
		target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY1);
		target_write_u32(target, PIC32MZ_NVMKEY, NVMKEY2);
		/* write back nvmpwp */
		target_write_u32(target, PIC32MZ_NVMPWP, nvmpwp);
	}

	return ERROR_OK;
}

/* see contib/loaders/flash/pic32mz.s for src */
static uint32_t pic32mz_flash_write_code[] = {
					/* write: */
	/* nvmkey1 = t0 = 0xAA996655 */
	0x3C08AA99,		/* lui $t0, 0xaa99 */
	0x35086655,		/* ori $t0, 0x6655 */
	/* nvmkey2 = t1 = 0x556699aa */
	0x3C095566,		/* lui $t1, 0x5566 */
	0x352999AA,		/* ori $t1, 0x99aa */
	/* nvmcon_addr = t2 = 0xBF800600 */
	0x3C0ABF80,		/* lui $t2, 0xbf80 */
	0x354A0600,		/* ori $t2, 0x0600 */
	/* nvmcon_value = t3 = 0x4003 */
	0x340B4003,		/* ori $t3, $zero, 0x4003 */
	/* nvmconset_value = t4 = 0x8000 */
	0x340C8000,		/* ori $t4, $zero, 0x8000 */
					/* write_row: */
	/* while (1) { */
	/* nvmconclr_value = t5 = 0x4000 */
	/*   if (param_words_left < 512) {
	     goto write_word;
	     }
	*/
	0x2CD30200,		/* sltiu $s3, $a2, 512; modified for MZ -- 512 words */
	0x16600008,		/* bne $s3, $zero, write_word */
	0x340D4000,		/* ori $t5, $zero, 0x4000 */
	/* *nvmaddr_addr = param_dest_address; */
	0xAD450020,		/* sw $a1, 32($t2) */
	/* *srcaddr_addr = param_source_address; */
	0xAD440070,		/* sw $a0, 112($t2) */
	/* param_source_address += 2048 */
	/* progflash() */
	0x04110016,		/* bal progflash */
	0x24840800,		/* addiu $a0, $a0, 2048 */
	/* param_dest_address += 2048; */
	0x24A50800,		/* addiu $a1, $a1, 2048 */
	/* param_words_left -= 512;
	 } end while */
	0x1000FFF7,		/* beq $zero, $zero, write_row */
	0x24C6FE00,		/* addiu $a2, $a2, -512 */
					/* write_word: */
	/* param_source_address |= 0xA0000000; */
	0x3C15A000,		/* lui $s5, 0xa000 */
	0x36B50000,		/* ori $s5, $s5, 0x0 */
	0x00952025,		/* or $a0, $a0, $s5 */
	/* nvmcon_value = 0x4001; */
	/* goto next_word; */
	0x10000008,		/* beq $zero, $zero, next_word */
	0x340B4001,		/* ori $t3, $zero, 0x4001 */
					/* prog_word: */
	/* do { */
	/*    *nvmdata_addr = *param_source_address; */
	0x8C940000,		/* lw $s4, 0($a0) */
	0xAD540030,		/* sw $s4, 48($t2) */
	/*    *nvmaddr_addr = param_dest_address; */
	0xAD450020,		/* sw $a1, 32($t2) */
	/*    param_source_address += 4; */
	/*    progflash() */
	0x04110009,		/* bal progflash */
	0x24840004,		/* addiu $a0, $a0, 4 */
	/*    param_dest_address += 4; */
	0x24A50004,		/* addiu $a1, $a1, 4 */
	/*    param_words_left -= 1; */
	0x24C6FFFF,		/* addiu $a2, $a2, -1 */
					/* next_word: */
	/* } while (param_words_left != 0); */
	0x14C0FFF8,		/* bne $a2, $zero, prog_word */
	0x00000000,		/* nop */
					/* done: */
	/* param_result = 0; */
	/* goto exit; */
	0x10000002,		/* beq $zero, $zero, exit */
	0x24040000,		/* addiu $a0, $zero, 0 */
					/* error: */
	/* param_result = nvmcon_result; */
	0x26240000,		/* addiu $a0, $s1, 0 */
					/* exit: */
	0x7000003F,		/* sdbbp */
					/* progflash: */
	/* *nvmcon_addr = nvmcon_value; */
	0xAD4B0000,		/* sw $t3, 0($t2) */
	/* *nvmkey_addr = nvmkey1; */
	0xAD480010,		/* sw $t0, 16($t2) */
	/* *nvmkey_addr = nvmkey2; */
	0xAD490010,		/* sw $t1, 16($t2) */
	/* *nvmconset_addr = nvmconset_value; */
	0xAD4C0008,		/* sw $t4, 8($t2) */
					/* waitflash: */
	/*    do { } while ((*NVMCON & nvmconset_value) != 0); */
	0x8D500000,		/* lw $s0, 0($t2) */
	0x020C8024,		/* and $s0, $s0, $t4 */
	0x1600FFFD,		/* bne $s0, $zero, waitflash */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	0x00000000,		/* nop */
	/*    if ((*NVMCON & 0x3000) != 0) {
		      goto error;
	      }
	*/
	0x8D510000,		/* lw $s1, 0($t2) */
	0x32313000,		/* andi $s1, $s1, 0x3000 */
	0x1620FFEF,		/* bne $s1, $zero, error */
	/* *nvmconclr_addr = nvmconclr_value; */
	0xAD4D0004,		/* sw $t5, 4($t2) */
	/* return */
	0x03E00008,		/* jr $ra */
	0x00000000		/* nop */
};

static int pic32mz_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[3];
	uint32_t row_size;
	int retval = ERROR_OK;

	// struct pic32mz_flash_bank *pic32mz_info = bank->driver_priv;
	struct mips32_algorithm mips32_info;

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(pic32mz_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* 2K byte row */
	row_size = 2*1024;

	uint8_t code[sizeof(pic32mz_flash_write_code)];
	target_buffer_set_u32_array(target, code,
				    ARRAY_SIZE(pic32mz_flash_write_code),
				    pic32mz_flash_write_code);
	retval = target_write_buffer(target, write_algorithm->address,
				     sizeof(code), code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	mips32_info.common_magic = MIPS32_COMMON_MAGIC;
	mips32_info.isa_mode = MIPS32_ISA_MIPS32;

	init_reg_param(&reg_params[0], "r4", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r5", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r6", 32, PARAM_OUT);

	int row_offset = offset % row_size;
	uint8_t *new_buffer = NULL;
	if (row_offset && (count >= (row_size / 4))) {
		new_buffer = malloc(buffer_size);
		if (new_buffer == NULL) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		memset(new_buffer,  0xff, row_offset);
		address -= row_offset;
	} else
		row_offset = 0;

	while (count > 0) {
		uint32_t status;
		uint32_t thisrun_count;

		if (row_offset) {
			thisrun_count = (count > ((buffer_size - row_offset) / 4)) ?
				((buffer_size - row_offset) / 4) : count;

			memcpy(new_buffer + row_offset, buffer, thisrun_count * 4);

			retval = target_write_buffer(target, source->address,
				row_offset + thisrun_count * 4, new_buffer);
			if (retval != ERROR_OK)
				break;
		} else {
			thisrun_count = (count > (buffer_size / 4)) ?
					(buffer_size / 4) : count;

			retval = target_write_buffer(target, source->address,
					thisrun_count * 4, buffer);
			if (retval != ERROR_OK)
				break;
		}

		buf_set_u32(reg_params[0].value, 0, 32, VIRT2PHYS(source->address));
		buf_set_u32(reg_params[1].value, 0, 32, VIRT2PHYS(address));
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count + row_offset / 4);

		retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
				write_algorithm->address,
				0, 10000, &mips32_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("error executing pic32mz flash write algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		status = buf_get_u32(reg_params[0].value, 0, 32);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count * 4;
		address += thisrun_count * 4;
		count -= thisrun_count;
		if (row_offset) {
			address += row_offset;
			row_offset = 0;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	if (new_buffer != NULL)
		free(new_buffer);
	return retval;
}

static int pic32mz_write_word(struct flash_bank *bank, uint32_t address,
			      uint32_t word)
{
	struct target *target = bank->target;

	target_write_u32(target, PIC32MZ_NVMADDR, VIRT2PHYS(address));
	target_write_u32(target, PIC32MZ_NVMDATA0, word);

	return pic32mz_nvm_exec(bank, NVMCON_OP_WORD_PROG, 5);
}

static int pic32mz_write_quad_word(struct flash_bank *bank, uint32_t address,
				   uint32_t word0, uint32_t word1,
				   uint32_t word2, uint32_t word3)
{
	struct target *target = bank->target;

	target_write_u32(target, PIC32MZ_NVMADDR, VIRT2PHYS(address));
	target_write_u32(target, PIC32MZ_NVMDATA0, word0);
	target_write_u32(target, PIC32MZ_NVMDATA1, word1);
	target_write_u32(target, PIC32MZ_NVMDATA2, word2);
	target_write_u32(target, PIC32MZ_NVMDATA3, word3);

	return pic32mz_nvm_exec(bank, NVMCON_OP_QUAD_WORD_PROG, 5);
}

static int pic32mz_write_quad_words(struct flash_bank *bank, uint32_t address,
				    const uint8_t *buffer, uint32_t numwords)
{
	uint32_t words_remaining = numwords;
	uint32_t curaddr = address;
	int status;

	if (address & 0xF) {
		LOG_WARNING("address 0x%" PRIx32 "breaks required 16-byte alignment for PIC32MZ quad-word programming", address);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	while (words_remaining != 0) {
		uint32_t chunk_words = (words_remaining > 4 ? 4 : words_remaining);
		uint32_t words[4];

		for (uint32_t i = 0; i < 4; i++) {
			if (chunk_words > i) {
				memcpy(&words[i],
				       buffer + (curaddr-address) + (i * 4),
				       sizeof(uint32_t));
			} else {
				words[i] = 0xFFFFFFFF;
			}
		}

		status = pic32mz_write_quad_word(bank, curaddr, words[0],
						 words[1], words[2], words[3]);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		curaddr += chunk_words * 4;
		words_remaining -= chunk_words;
	}

	return ERROR_OK;
}

static int pic32mz_write(struct flash_bank *bank, const uint8_t *buffer,
			 uint32_t offset, uint32_t count)
{
	uint32_t words_remaining = (count / 4);
	uint32_t bytes_remaining = (count & 0x00000003);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	uint32_t status;
	int retval;
	uint32_t cfgcon_address;
	uint32_t cfgcon;
	bool ecc_is_on;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("writing to flash at address " TARGET_ADDR_FMT " at offset 0x%8.8" PRIx32
			" count: 0x%8.8" PRIx32 "", bank->base, offset, count);

	cfgcon_address = PIC32MZ_CFGCON;
	target_read_u32(bank->target, cfgcon_address, &cfgcon);
	ecc_is_on = ((cfgcon & CFGCON_ECC) == 0);

	if (ecc_is_on || VIRT2PHYS(bank->base) == PIC32MZ_PHYS_LOWER_BOOT_ALIAS
	    || VIRT2PHYS(bank->base) == PIC32MZ_PHYS_UPPER_BOOT_ALIAS) {
		return pic32mz_write_quad_words(bank, address, buffer, count/4);
	}

	if (offset & 0x3) {
		LOG_WARNING("offset 0x%" PRIx32 "breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* multiple words (4-byte) to be programmed? */
	if (words_remaining > 0) {
		/* try using a block write */
		retval = pic32mz_write_block(bank, buffer, offset,
					     words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			} else if (retval == ERROR_FLASH_OPERATION_FAILED) {
				LOG_ERROR("flash writing failed");
				return retval;
			}
		} else {
			buffer += words_remaining * 4;
			address += words_remaining * 4;
			words_remaining = 0;
		}
	}

	while (words_remaining > 0) {
		uint32_t value;
		memcpy(&value, buffer + bytes_written, sizeof(uint32_t));

		status = pic32mz_write_word(bank, address, value);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bytes_written += 4;
		words_remaining--;
		address += 4;
	}

	if (bytes_remaining) {
		uint32_t value = 0xffffffff;
		memcpy(&value, buffer + bytes_written, bytes_remaining);

		status = pic32mz_write_word(bank, address, value);

		if (status & NVMCON_NVMERR) {
			LOG_ERROR("Flash write error NVMERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (status & NVMCON_LVDERR) {
			LOG_ERROR("Flash write error LVDERR (status = 0x%08" PRIx32 ")", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int pic32mz_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct pic32mz_flash_bank *pic32mz_info = bank->driver_priv;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int i;
	uint32_t num_pages = 0;
	uint32_t num_bytes = 0;
	uint32_t device_id;
	int page_size;
	const struct pic32mz_devs_s *pdev = NULL;

	pic32mz_info->probed = 0;

	device_id = ejtag_info->idcode;
	LOG_INFO("device id = 0x%08" PRIx32 " (manuf 0x%03x dev 0x%04x, ver 0x%02x)",
		 device_id,
		 (unsigned)((device_id >> 1) & 0x7ff),
		 (unsigned)((device_id >> 12) & 0xffff),
		 (unsigned)((device_id >> 28) & 0xf));

	if (((device_id >> 1) & 0x7ff) != PIC32MZ_MANUF_ID) {
		LOG_WARNING("Cannot identify target as a PIC32MZ family.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	page_size = 16*1024;  // fixed size pages for PIC32MZ

	if (VIRT2PHYS(bank->base) == PIC32MZ_PHYS_LOWER_BOOT_ALIAS
	    || VIRT2PHYS(bank->base) == PIC32MZ_PHYS_UPPER_BOOT_ALIAS) {
		/* 0x1FC00000 or 0x1FC20000: Boot flash size */
		num_bytes = PIC32MZ_BOOT_BLOCK_NUMBYTES;
	} else if ((pdev = pic32mz_lookup_device(device_id)) != NULL) {
		num_bytes = (pdev->kb_program_flash * 1024);
	} else {
		/* unknown device, assume 512 KB program flash */
		num_bytes = (512 * 1024);
	}

	LOG_INFO("flash size = %" PRId32 "kbytes", num_bytes / 1024);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* calculate numbers of pages */
	num_pages = num_bytes / page_size;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < (int)num_pages; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	pic32mz_protect_check(bank);

	pic32mz_info->probed = 1;

	return ERROR_OK;
}

static int pic32mz_auto_probe(struct flash_bank *bank)
{
	struct pic32mz_flash_bank *pic32mz_info = bank->driver_priv;
	if (pic32mz_info->probed)
		return ERROR_OK;
	return pic32mz_probe(bank);
}

static int pic32mz_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	struct mips32_common *mips32 = target->arch_info;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	uint32_t device_id;
	int printed = 0, i;

	device_id = ejtag_info->idcode;

	if (((device_id >> 1) & 0x7ff) != PIC32MZ_MANUF_ID) {
		snprintf(buf, buf_size,
			 "Cannot identify target as a PIC32MZ family (manufacturer 0x%03d != 0x%03d)\n",
			 (unsigned)((device_id >> 1) & 0x7ff),
			 PIC32MZ_MANUF_ID);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	for (i = 0; pic32mz_devs[i].name != NULL; i++) {
		if (pic32mz_devs[i].devid == (device_id & 0x0fffffff)) {
			printed = snprintf(buf, buf_size, "PIC32MZ%s",
					   pic32mz_devs[i].name);
			break;
		}
	}

	if (pic32mz_devs[i].name == NULL)
		printed = snprintf(buf, buf_size, "Unknown");

	buf += printed;
	buf_size -= printed;
	snprintf(buf, buf_size, " Ver: 0x%02x",
			(unsigned)((device_id >> 28) & 0xf));

	return ERROR_OK;
}

COMMAND_HANDLER(pic32mz_handle_pgm_word_command)
{
	uint32_t address, value;
	int status, res;

	if (CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 2, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (address < bank->base || address >= (bank->base + bank->size)) {
		command_print(CMD, "flash address '%s' is out of bounds",
			      CMD_ARGV[0]);
		return ERROR_OK;
	}

	res = ERROR_OK;
	status = pic32mz_write_word(bank, address, value);
	if (status & NVMCON_NVMERR)
		res = ERROR_FLASH_OPERATION_FAILED;
	if (status & NVMCON_LVDERR)
		res = ERROR_FLASH_OPERATION_FAILED;

	if (res == ERROR_OK)
		command_print(CMD, "pic32mz pgm word complete");
	else
		command_print(CMD, "pic32mz pgm word failed (status = 0x%x)",
			      status);

	return ERROR_OK;
}

COMMAND_HANDLER(pic32mz_handle_unlock_command)
{
	struct target *target = NULL;
	struct mips_m4k_common *mips_m4k;
	struct mips_ejtag *ejtag_info;
	int timeout = 10;

	if (CMD_ARGC < 1) {
		command_print(CMD, "pic32mz unlock <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	mips_m4k = target_to_m4k(target);
	ejtag_info = &mips_m4k->mips32.ejtag_info;

	/* we have to use the MTAP to perform a full erase */
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_MTAP);
	mips_ejtag_set_instr(ejtag_info, MTAP_COMMAND);

	/* first check status of device */
	uint8_t mchip_cmd = MCHP_STATUS;
	mips_ejtag_drscan_8(ejtag_info, &mchip_cmd);
	if (mchip_cmd & (1 << 7)) {
		/* device is not locked */
		command_print(CMD, "pic32mz is already unlocked, erasing anyway");
	}

	/* unlock/erase device */
	mips_ejtag_drscan_8_out(ejtag_info, MCHP_ASERT_RST);
	jtag_add_sleep(200);

	mips_ejtag_drscan_8_out(ejtag_info, MCHP_ERASE);

	do {
		mchip_cmd = MCHP_STATUS;
		mips_ejtag_drscan_8(ejtag_info, &mchip_cmd);
		if (timeout-- == 0) {
			LOG_DEBUG("timeout waiting for unlock: 0x%" PRIx32 "",
				  mchip_cmd);
			break;
		}
		alive_sleep(1);
	} while ((mchip_cmd & (1 << 2)) || (!(mchip_cmd & (1 << 3))));

	mips_ejtag_drscan_8_out(ejtag_info, MCHP_DE_ASSERT_RST);

	/* select ejtag tap */
	mips_ejtag_set_instr(ejtag_info, MTAP_SW_ETAP);

	command_print(CMD, "pic32mz unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

static const struct command_registration pic32mz_exec_command_handlers[] = {
	{
		.name = "pgm_word",
		.usage = "<addr> <value> <bank>",
		.handler = pic32mz_handle_pgm_word_command,
		.mode = COMMAND_EXEC,
		.help = "program a word",
	},
	{
		.name = "unlock",
		.handler = pic32mz_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "[bank_id]",
		.help = "Unlock/Erase entire device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration pic32mz_command_handlers[] = {
	{
		.name = "pic32mz",
		.mode = COMMAND_ANY,
		.help = "pic32mz flash command group",
		.usage = "",
		.chain = pic32mz_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver pic32mz_flash = {
	.name = "pic32mz",
	.commands = pic32mz_command_handlers,
	.flash_bank_command = pic32mz_flash_bank_command,
	.erase = pic32mz_erase,
	.protect = pic32mz_protect,
	.write = pic32mz_write,
	.read = default_flash_read,
	.probe = pic32mz_probe,
	.auto_probe = pic32mz_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pic32mz_protect_check,
	.info = pic32mz_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
