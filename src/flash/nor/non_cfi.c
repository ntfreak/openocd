/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2009 Michael Schwingen                                  *
 *   michael@schwingen.org                                                 *
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
#include "cfi.h"
#include "non_cfi.h"

#define KB 1024
#define MB (1024*1024)
#define ERASE_REGION(num, size) (((size/256) << 16) | (num-1))

/* non-CFI compatible flashes */
static const struct non_cfi non_cfi_flashes[] = {
	{
		.mfr = CFI_MFR_SST,
		.id = 0xd4,
		.pri_id = 0x02,
		.dev_size = 64*KB,
		.interface_desc = 0x0,		/* x8 only device */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(16, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0xd5,
		.pri_id = 0x02,
		.dev_size = 128*KB,
		.interface_desc = 0x0,		/* x8 only device */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(32, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0xd6,
		.pri_id = 0x02,
		.dev_size = 256*KB,
		.interface_desc = 0x0,		/* x8 only device */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(64, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0xd7,
		.pri_id = 0x02,
		.dev_size = 512*KB,
		.interface_desc = 0x0,		/* x8 only device */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(128, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_AMD,		/* Spansion AM29LV040B */
		.id = 0x4f,
		.pri_id = 0x02,
		.dev_size = 512*KB,
		.interface_desc = 0x0,		/* x8 only device */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(8, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0x2780,
		.pri_id = 0x02,
		.dev_size = 512*KB,
		.interface_desc = 0x2,		/* x8 or x16 device */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(128, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_ST,
		.id = 0xd6,					/* ST29F400BB */
		.pri_id = 0x02,
		.dev_size = 512*KB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(7, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_ST,
		.id = 0xd5,					/* ST29F400BT */
		.pri_id = 0x02,
		.dev_size = 512*KB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(7, 64*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1, 16*KB)
		}
	},

	/* SST 39VF* do not support DQ5 status polling - this currently is
	   only supported by the host algorithm, not by the target code using
	   the work area.
	   Only true for 8-bit and 32-bit wide memories. 16-bit wide memories
	   without DQ5 status polling are supported by the target code.
	*/
	{
		.mfr = CFI_MFR_SST,
		.id = 0x2782,				/* SST39xF160 */
		.pri_id = 0x02,
		.dev_size = 2*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(512, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0x2783,				/* SST39VF320 */
		.pri_id = 0x02,
		.dev_size = 4*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(1024, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0x234b,				/* SST39VF1601 */
		.pri_id = 0x02,
		.dev_size = 2*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(512, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0x274b,				/* SST39WF1601 */
		.pri_id = 0x02,
		.dev_size = 2*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(512, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0x234a,				/* SST39VF1602 */
		.pri_id = 0x02,
		.dev_size = 2*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(512, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0x235b,				/* SST39VF3201 */
		.pri_id = 0x02,
		.dev_size = 4*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(1024, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0x235a,				/* SST39VF3202 */
		.pri_id = 0x02,
		.dev_size = 4*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(1024, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_SST,
		.id = 0x236d,		/* SST39VF6401B */
		.pri_id = 0x02,
		.dev_size = 8*MB,
		.interface_desc = 0x2,	/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ6_DQ7,
		.num_erase_regions = 1,
		.erase_region_info = {
			ERASE_REGION(2048, 4*KB)
		}
	},
	{
		.mfr = CFI_MFR_AMD,
		.id = 0x22ab,				/* AM29F400BB */
		.pri_id = 0x02,
		.dev_size = 512*KB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(7, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_AMD,
		.id = 0x2223,				/* AM29F400BT */
		.pri_id = 0x02,
		.dev_size = 512*KB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(7, 64*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1, 16*KB)
		}
	},
	{
		.mfr = CFI_MFR_FUJITSU,
		.id = 0x226b,				/* AM29SL800DB */
		.pri_id = 0x02,
		.dev_size = 1*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(15, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_FUJITSU,
		.id = 0x22ea,				/* MBM29SL800TE */
		.pri_id = 0x02,
		.dev_size = 1*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(15, 64*KB),
			ERASE_REGION(1,  32*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1,  16*KB)
		}
	},
	{
		.mfr = CFI_MFR_FUJITSU,
		.id = 0xba,				/* 29LV400BC */
		.pri_id = 0x02,
		.dev_size = 512*KB,
		.interface_desc = 0x1,		/* x8 or x16 device w/ nBYTE */
		.max_buf_write_size = 0x00,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(7, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_AMIC,
		.id = 0xb31a,				/* A29L800A */
		.pri_id = 0x02,
		.dev_size = 1*MB,
		.interface_desc = 0x2,
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(15, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_MX,
		.id = 0x225b,				/* MX29LV800B */
		.pri_id = 0x02,
		.dev_size = 1*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2, 8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(15, 64*KB)
		}
	},

	{
		.mfr = CFI_MFR_MX,
		.id = 0x2249,				/* MX29LV160AB: 2MB */
		.pri_id = 0x02,
		.dev_size = 2*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2, 8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(31, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_MX,
		.id = 0x22C4,				/* MX29LV160AT: 2MB */
		.pri_id = 0x02,
		.dev_size = 2*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(31, 64*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(2, 8*KB),
			ERASE_REGION(1, 16*KB)
		}
	},
	{
		.mfr = CFI_MFR_EON,
		.id = 0x225b,				/* EN29LV800BB */
		.pri_id = 0x02,
		.dev_size = 1*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2,  8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(15, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_ATMEL,
		.id = 0x00c0,				/* Atmel 49BV1614 */
		.pri_id = 0x02,
		.dev_size = 2*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 3,
		.erase_region_info = {
			ERASE_REGION(8,  8*KB),
			ERASE_REGION(2, 32*KB),
			ERASE_REGION(30, 64*KB)
		}
	},
	{
		.mfr = CFI_MFR_ATMEL,
		.id = 0xC2,					/* Atmel 49BV1614T */
		.pri_id = 0x02,
		.dev_size = 2*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 3,
		.erase_region_info = {
			ERASE_REGION(30, 64*KB),
			ERASE_REGION(2, 32*KB),
			ERASE_REGION(8,  8*KB)
		}
	},
	{
		.mfr = CFI_MFR_AMD,
		.id = 0x225b,				/* S29AL008D */
		.pri_id = 0x02,
		.dev_size = 1*MB,
		.interface_desc = 0x2,		/* x8 or x16 device with nBYTE */
		.max_buf_write_size = 0x0,
		.status_poll_mask = CFI_STATUS_POLL_MASK_DQ5_DQ6_DQ7,
		.num_erase_regions = 4,
		.erase_region_info = {
			ERASE_REGION(1, 16*KB),
			ERASE_REGION(2, 8*KB),
			ERASE_REGION(1, 32*KB),
			ERASE_REGION(15, 64*KB)
		}
	},
	{
		.mfr = 0,
		.id = 0,
	}
};

void cfi_fixup_non_cfi(struct flash_bank *bank)
{
	unsigned int mask;
	struct cfi_flash_bank *cfi_info = bank->driver_priv;
	const struct non_cfi *non_cfi = non_cfi_flashes;

	if (cfi_info->x16_as_x8)
		mask = 0xFF;
	else
		mask = 0xFFFF;

	for (non_cfi = non_cfi_flashes; non_cfi->mfr; non_cfi++) {
		if ((cfi_info->manufacturer == non_cfi->mfr)
		    && (cfi_info->device_id == (non_cfi->id & mask)))
			break;
	}

	/* only fixup jedec flashes found in table */
	if (!non_cfi->mfr)
		return;

	cfi_info->not_cfi = true;

	/* fill in defaults for non-critical data */
	cfi_info->vcc_min = 0x0;
	cfi_info->vcc_max = 0x0;
	cfi_info->vpp_min = 0x0;
	cfi_info->vpp_max = 0x0;
	/* these are used for timeouts - use vales that should be long enough
	   for normal operation. */
	cfi_info->word_write_timeout_typ = 0x0a;
	cfi_info->buf_write_timeout_typ = 0x0d;
	cfi_info->block_erase_timeout_typ = 0x0d;
	cfi_info->chip_erase_timeout_typ = 0x10;
	cfi_info->word_write_timeout_max = 0x0;
	cfi_info->buf_write_timeout_max = 0x0;
	cfi_info->block_erase_timeout_max = 0x0;
	cfi_info->chip_erase_timeout_max = 0x0;

	cfi_info->qry[0] = 'Q';
	cfi_info->qry[1] = 'R';
	cfi_info->qry[2] = 'Y';

	cfi_info->pri_id = non_cfi->pri_id;
	cfi_info->pri_addr = 0x0;
	cfi_info->alt_id = 0x0;
	cfi_info->alt_addr = 0x0;
	cfi_info->alt_ext = NULL;

	cfi_info->interface_desc = non_cfi->interface_desc;
	cfi_info->max_buf_write_size = non_cfi->max_buf_write_size;
	cfi_info->status_poll_mask = non_cfi->status_poll_mask;
	cfi_info->num_erase_regions = non_cfi->num_erase_regions;
	size_t erase_region_info_size = sizeof(*cfi_info->erase_region_info) *
		cfi_info->num_erase_regions;
	cfi_info->erase_region_info = malloc(erase_region_info_size);
	memcpy(cfi_info->erase_region_info,
		non_cfi->erase_region_info, erase_region_info_size);
	cfi_info->dev_size = non_cfi->dev_size;

	if (cfi_info->pri_id == 0x2) {
		struct cfi_spansion_pri_ext *pri_ext = malloc(sizeof(struct cfi_spansion_pri_ext));

		pri_ext->pri[0] = 'P';
		pri_ext->pri[1] = 'R';
		pri_ext->pri[2] = 'I';

		pri_ext->major_version = '1';
		pri_ext->minor_version = '0';

		pri_ext->SiliconRevision = 0x0;
		pri_ext->EraseSuspend = 0x0;
		pri_ext->BlkProt = 0x0;
		pri_ext->TmpBlkUnprotect = 0x0;
		pri_ext->BlkProtUnprot = 0x0;
		pri_ext->SimultaneousOps = 0x0;
		pri_ext->BurstMode = 0x0;
		pri_ext->PageMode = 0x0;
		pri_ext->VppMin = 0x0;
		pri_ext->VppMax = 0x0;
		pri_ext->TopBottom = 0x0;

		pri_ext->_unlock1 = 0x5555;
		pri_ext->_unlock2 = 0x2AAA;
		pri_ext->_reversed_geometry = 0;

		cfi_info->pri_ext = pri_ext;
	} else if ((cfi_info->pri_id == 0x1) || (cfi_info->pri_id == 0x3)) {
		LOG_ERROR("BUG: non-CFI flashes using the Intel commandset are not yet supported");
		exit(-1);
	}
}
