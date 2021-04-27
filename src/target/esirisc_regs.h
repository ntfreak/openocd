/***************************************************************************
 *   Copyright (C) 2018 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
 *   James Zhao <hjz@squareup.com>                                         *
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

#ifndef OPENOCD_TARGET_ESIRISC_REGS_H
#define OPENOCD_TARGET_ESIRISC_REGS_H

enum esirisc_reg_num {
	ESIRISC_SP,
	ESIRISC_RA,
	ESIRISC_R2,
	ESIRISC_R3,
	ESIRISC_R4,
	ESIRISC_R5,
	ESIRISC_R6,
	ESIRISC_R7,
	ESIRISC_R8,
	ESIRISC_R9,
	ESIRISC_R10,
	ESIRISC_R11,
	ESIRISC_R12,
	ESIRISC_R13,
	ESIRISC_R14,
	ESIRISC_R15,
	ESIRISC_R16,
	ESIRISC_R17,
	ESIRISC_R18,
	ESIRISC_R19,
	ESIRISC_R20,
	ESIRISC_R21,
	ESIRISC_R22,
	ESIRISC_R23,
	ESIRISC_R24,
	ESIRISC_R25,
	ESIRISC_R26,
	ESIRISC_R27,
	ESIRISC_R28,
	ESIRISC_R29,
	ESIRISC_R30,
	ESIRISC_R31,

	ESIRISC_V0,
	ESIRISC_V1,
	ESIRISC_V2,
	ESIRISC_V3,
	ESIRISC_V4,
	ESIRISC_V5,
	ESIRISC_V6,
	ESIRISC_V7,
	ESIRISC_V8,
	ESIRISC_V9,
	ESIRISC_V10,
	ESIRISC_V11,
	ESIRISC_V12,
	ESIRISC_V13,
	ESIRISC_V14,
	ESIRISC_V15,
	ESIRISC_V16,
	ESIRISC_V17,
	ESIRISC_V18,
	ESIRISC_V19,
	ESIRISC_V20,
	ESIRISC_V21,
	ESIRISC_V22,
	ESIRISC_V23,
	ESIRISC_V24,
	ESIRISC_V25,
	ESIRISC_V26,
	ESIRISC_V27,
	ESIRISC_V28,
	ESIRISC_V29,
	ESIRISC_V30,
	ESIRISC_V31,

	ESIRISC_A0,
	ESIRISC_A1,
	ESIRISC_A2,
	ESIRISC_A3,
	ESIRISC_A4,
	ESIRISC_A5,
	ESIRISC_A6,
	ESIRISC_A7,

	ESIRISC_PC,
	ESIRISC_CAS,
	ESIRISC_TC,
	ESIRISC_ETA,
	ESIRISC_ETC,
	ESIRISC_EPC,
	ESIRISC_ECAS,
	ESIRISC_EID,
	ESIRISC_ED,
	ESIRISC_IP,
	ESIRISC_IM,
	ESIRISC_IS,
	ESIRISC_IT,

	ESIRISC_NUM_REGS,
};

/* CSR Banks */
#define CSR_THREAD					0x00
#define CSR_INTERRUPT				0x01
#define CSR_DEBUG					0x04
#define CSR_CONFIG					0x05
#define CSR_TRACE					0x09

/* Thread CSRs */
#define CSR_THREAD_TC				0x00	/* Thread Control */
#define CSR_THREAD_PC				0x01	/* Program Counter */
#define CSR_THREAD_CAS				0x02	/* Comparison & Arithmetic Status */
#define CSR_THREAD_AC				0x03	/* Arithmetic Control */
#define CSR_THREAD_LF				0x04	/* Locked Flag */
#define CSR_THREAD_LA				0x05	/* Locked Address */
#define CSR_THREAD_ETA				0x07	/* Exception Table Address */
#define CSR_THREAD_ETC				0x08	/* Exception TC */
#define CSR_THREAD_EPC				0x09	/* Exception PC */
#define CSR_THREAD_ECAS				0x0a	/* Exception CAS */
#define CSR_THREAD_EID				0x0b	/* Exception ID */
#define CSR_THREAD_ED				0x0c	/* Exception Data */

/* Interrupt CSRs */
#define CSR_INTERRUPT_IP			0x00	/* Interrupt Pending */
#define CSR_INTERRUPT_IA			0x01	/* Interrupt Acknowledge */
#define CSR_INTERRUPT_IM			0x02	/* Interrupt Mask */
#define CSR_INTERRUPT_IS			0x03	/* Interrupt Sense */
#define CSR_INTERRUPT_IT			0x04	/* Interrupt Trigger */

/* Debug CSRs */
#define CSR_DEBUG_DC				0x00	/* Debug Control */
#define CSR_DEBUG_IBC				0x01	/* Instruction Breakpoint Control */
#define CSR_DEBUG_DBC				0x02	/* Data Breakpoint Control */
#define CSR_DEBUG_HWDC				0x03	/* Hardware Debug Control */
#define CSR_DEBUG_DBS				0x04	/* Data Breakpoint Size */
#define CSR_DEBUG_DBR				0x05	/* Data Breakpoint Range */
#define CSR_DEBUG_IBA_N				0x08	/* Instruction Breakpoint Address [0..7] */
#define CSR_DEBUG_DBA_N				0x10	/* Data Breakpoint Address [0..7] */

/* Configuration CSRs */
#define CSR_CONFIG_ARCH0			0x00	/* Architectural Configuration 0 */
#define CSR_CONFIG_ARCH1			0x01	/* Architectural Configuration 1 */
#define CSR_CONFIG_ARCH2			0x02	/* Architectural Configuration 2 */
#define CSR_CONFIG_ARCH3			0x03	/* Architectural Configuration 3 */
#define CSR_CONFIG_MEM				0x04	/* Memory Configuration */
#define CSR_CONFIG_IC				0x05	/* Instruction Cache Configuration */
#define CSR_CONFIG_DC				0x06	/* Data Cache Configuration */
#define CSR_CONFIG_INT				0x07	/* Interrupt Configuration */
#define	CSR_CONFIG_ISA_N			0x08	/* Instruction Set Configuration [0..6] */
#define CSR_CONFIG_DBG				0x0f	/* Debug Configuration */
#define CSR_CONFIG_MID				0x10	/* Manufacturer ID */
#define CSR_CONFIG_REV				0x11	/* Revision Number */
#define CSR_CONFIG_MPID				0x12	/* Multiprocessor ID */
#define CSR_CONFIG_FREQ_N			0x13	/* Frequency [0..2] */
#define CSR_CONFIG_TRACE			0x16	/* Trace Configuration */

/* Trace CSRs */
#define CSR_TRACE_CONTROL			0x00
#define CSR_TRACE_STATUS			0x01
#define CSR_TRACE_BUFFER_START		0x02
#define CSR_TRACE_BUFFER_END		0x03
#define CSR_TRACE_BUFFER_CUR		0x04
#define CSR_TRACE_TRIGGER			0x05
#define CSR_TRACE_START_DATA		0x06
#define CSR_TRACE_START_MASK		0x07
#define CSR_TRACE_STOP_DATA			0x08
#define CSR_TRACE_STOP_MASK			0x09
#define CSR_TRACE_DELAY				0x0a

#endif /* OPENOCD_TARGET_ESIRISC_REGS_H */
