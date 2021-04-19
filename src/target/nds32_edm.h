/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#ifndef OPENOCD_TARGET_NDS32_EDM_H
#define OPENOCD_TARGET_NDS32_EDM_H

#include "helper/types.h"

/**
 * @file
 * This is the interface to the Embedded Debug Module for Andes cores.
 */

/* EDM misc registers */
enum nds_edm_misc_reg {
	NDS_EDM_MISC_DIMIR = 0x0,
	NDS_EDM_MISC_SBAR,
	NDS_EDM_MISC_EDM_CMDR,
	NDS_EDM_MISC_DBGER,
	NDS_EDM_MISC_ACC_CTL,
	NDS_EDM_MISC_EDM_PROBE,
	NDS_EDM_MISC_GEN_PORT0,
	NDS_EDM_MISC_GEN_PORT1,
};

/* EDM system registers */
enum nds_edm_system_reg {
	NDS_EDM_SR_BPC0 = 0x00,
	NDS_EDM_SR_BPC1,
	NDS_EDM_SR_BPC2,
	NDS_EDM_SR_BPC3,
	NDS_EDM_SR_BPC4,
	NDS_EDM_SR_BPC5,
	NDS_EDM_SR_BPC6,
	NDS_EDM_SR_BPC7,
	NDS_EDM_SR_BPA0 = 0x08,
	NDS_EDM_SR_BPA1,
	NDS_EDM_SR_BPA2,
	NDS_EDM_SR_BPA3,
	NDS_EDM_SR_BPA4,
	NDS_EDM_SR_BPA5,
	NDS_EDM_SR_BPA6,
	NDS_EDM_SR_BPA7,
	NDS_EDM_SR_BPAM0 = 0x10,
	NDS_EDM_SR_BPAM1,
	NDS_EDM_SR_BPAM2,
	NDS_EDM_SR_BPAM3,
	NDS_EDM_SR_BPAM4,
	NDS_EDM_SR_BPAM5,
	NDS_EDM_SR_BPAM6,
	NDS_EDM_SR_BPAM7,
	NDS_EDM_SR_BPV0 = 0x18,
	NDS_EDM_SR_BPV1,
	NDS_EDM_SR_BPV2,
	NDS_EDM_SR_BPV3,
	NDS_EDM_SR_BPV4,
	NDS_EDM_SR_BPV5,
	NDS_EDM_SR_BPV6,
	NDS_EDM_SR_BPV7,
	NDS_EDM_SR_BPCID0 = 0x20,
	NDS_EDM_SR_BPCID1,
	NDS_EDM_SR_BPCID2,
	NDS_EDM_SR_BPCID3,
	NDS_EDM_SR_BPCID4,
	NDS_EDM_SR_BPCID5,
	NDS_EDM_SR_BPCID6,
	NDS_EDM_SR_BPCID7,
	NDS_EDM_SR_EDM_CFG = 0x28,
	NDS_EDM_SR_EDMSW = 0x30,
	NDS_EDM_SR_EDM_CTL = 0x38,
	NDS_EDM_SR_EDM_DTR = 0x40,
	NDS_EDM_SR_BPMTV = 0x48,
	NDS_EDM_SR_DIMBR = 0x50,
	NDS_EDM_SR_TECR0 = 0x70,
	NDS_EDM_SR_TECR1 = 0x71,
};

enum nds_memory_access {
	NDS_MEMORY_ACC_BUS = 0,
	NDS_MEMORY_ACC_CPU,
};

enum nds_memory_select {
	NDS_MEMORY_SELECT_AUTO = 0,
	NDS_MEMORY_SELECT_MEM = 1,
	NDS_MEMORY_SELECT_ILM = 2,
	NDS_MEMORY_SELECT_DLM = 3,
};

#define NDS_DBGER_DEX		(0x1)
#define NDS_DBGER_DPED		(0x2)
#define NDS_DBGER_CRST		(0x4)
#define NDS_DBGER_AT_MAX	(0x8)
#define NDS_DBGER_ILL_SEC_ACC	(0x10)
#define NDS_DBGER_ALL_SUPRS_EX	(0x40000000)
#define NDS_DBGER_RESACC	(0x80000000)
#define NDS_DBGER_CLEAR_ALL	(0x1F)

#define NDS_EDMSW_WDV		(1 << 0)
#define NDS_EDMSW_RDV		(1 << 1)

#endif /* OPENOCD_TARGET_NDS32_EDM_H */
