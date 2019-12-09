/*
 * Copyright (C) 2015 by Matthias Welwarsky <matthias.welwarsky@sysgo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>
#include <stdbool.h>

#include "armv8.h"
#include "armv8_opcodes.h"

static const uint32_t a64_opcodes[ARMV8_OPC_NUM] = {
		[READ_REG_CTR]		= ARMV8_MRS(SYSTEM_CTR, 0),
		[READ_REG_CLIDR]	= ARMV8_MRS(SYSTEM_CLIDR, 0),
		[READ_REG_CSSELR]	= ARMV8_MRS(SYSTEM_CSSELR, 0),
		[READ_REG_CCSIDR]	= ARMV8_MRS(SYSTEM_CCSIDR, 0),
		[WRITE_REG_CSSELR]	= ARMV8_MSR_GP(SYSTEM_CSSELR, 0),
		[READ_REG_MPIDR]	= ARMV8_MRS(SYSTEM_MPIDR, 0),
		[READ_REG_DTRRX]	= ARMV8_MRS(SYSTEM_DBG_DTRRX_EL0, 0),
		[WRITE_REG_DTRTX]	= ARMV8_MSR_GP(SYSTEM_DBG_DTRTX_EL0, 0),
		[WRITE_REG_DSPSR]	= ARMV8_MSR_DSPSR(0),
		[READ_REG_DSPSR]	= ARMV8_MRS_DSPSR(0),
		[ARMV8_OPC_DSB_SY]	= ARMV8_DSB_SY,
		[ARMV8_OPC_DCPS]	= ARMV8_DCPS(0, 11),
		[ARMV8_OPC_DRPS]	= ARMV8_DRPS,
		[ARMV8_OPC_ISB_SY]	= ARMV8_ISB,
		[ARMV8_OPC_DCCISW]	= ARMV8_SYS(SYSTEM_DCCISW, 0),
		[ARMV8_OPC_DCCIVAC]	= ARMV8_SYS(SYSTEM_DCCIVAC, 0),
		[ARMV8_OPC_ICIVAU]	= ARMV8_SYS(SYSTEM_ICIVAU, 0),
		[ARMV8_OPC_HLT]		= ARMV8_HLT(11),
		[ARMV8_OPC_LDRB_IP]	= ARMV8_LDRB_IP(1, 0),
		[ARMV8_OPC_LDRH_IP]	= ARMV8_LDRH_IP(1, 0),
		[ARMV8_OPC_LDRW_IP]	= ARMV8_LDRW_IP(1, 0),
		[ARMV8_OPC_STRB_IP]	= ARMV8_STRB_IP(1, 0),
		[ARMV8_OPC_STRH_IP]	= ARMV8_STRH_IP(1, 0),
		[ARMV8_OPC_STRW_IP]	= ARMV8_STRW_IP(1, 0),
};

static const uint32_t t32_opcodes[ARMV8_OPC_NUM] = {
		[READ_REG_CTR]		= ARMV4_5_MRC(15, 0, 0, 0, 0, 1),
		[READ_REG_CLIDR]	= ARMV4_5_MRC(15, 1, 0, 0, 0, 1),
		[READ_REG_CSSELR]	= ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
		[READ_REG_CCSIDR]	= ARMV4_5_MRC(15, 1, 0, 0, 0, 0),
		[WRITE_REG_CSSELR]	= ARMV4_5_MCR(15, 2, 0, 0, 0, 0),
		[READ_REG_MPIDR]	= ARMV4_5_MRC(15, 0, 0, 0, 0, 5),
		[READ_REG_DTRRX]	= ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
		[WRITE_REG_DTRTX]	= ARMV4_5_MCR(14, 0, 0, 0, 5, 0),
		[WRITE_REG_DSPSR]	= ARMV8_MCR_DSPSR(0),
		[READ_REG_DSPSR]	= ARMV8_MRC_DSPSR(0),
		[ARMV8_OPC_DSB_SY]	= ARMV8_DSB_SY_T1,
		[ARMV8_OPC_DCPS]	= ARMV8_DCPS_T1(0),
		[ARMV8_OPC_DRPS]	= ARMV8_ERET_T1,
		[ARMV8_OPC_ISB_SY]	= ARMV8_ISB_SY_T1,
		[ARMV8_OPC_DCCISW]	= ARMV4_5_MCR(15, 0, 0, 7, 14, 2),
		[ARMV8_OPC_DCCIVAC]	= ARMV4_5_MCR(15, 0, 0, 7, 14, 1),
		[ARMV8_OPC_ICIVAU]	= ARMV4_5_MCR(15, 0, 0, 7, 5, 1),
		[ARMV8_OPC_HLT]		= ARMV8_HLT_T1(11),
		[ARMV8_OPC_LDRB_IP]	= ARMV8_LDRB_IP_T3(1, 0),
		[ARMV8_OPC_LDRH_IP]	= ARMV8_LDRH_IP_T3(1, 0),
		[ARMV8_OPC_LDRW_IP]	= ARMV8_LDRW_IP_T3(1, 0),
		[ARMV8_OPC_STRB_IP]	= ARMV8_STRB_IP_T3(1, 0),
		[ARMV8_OPC_STRH_IP]	= ARMV8_STRH_IP_T3(1, 0),
		[ARMV8_OPC_STRW_IP]	= ARMV8_STRW_IP_T3(1, 0),
};

void armv8_select_opcodes(struct armv8_common *armv8, bool state_is_aarch64)
{
	if (state_is_aarch64)
		armv8->opcodes = &a64_opcodes[0];
	else
		armv8->opcodes = &t32_opcodes[0];
}

uint32_t armv8_opcode(struct armv8_common *armv8, enum armv8_opcode code)
{
	if ((int)code >= ARMV8_OPC_NUM)
		return -1;

	return *(armv8->opcodes + code);
}
