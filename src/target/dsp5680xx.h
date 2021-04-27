/***************************************************************************
 *   Copyright (C) 2011 by Rodrigo L. Rosa                                 *
 *   rodrigorosa.LG@gmail.com                                              *
 *                                                                         *
 *   Based on dsp563xx_once.h written by Mathias Kuester                   *
 *   mkdorg@users.sourceforge.net                                          *
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

#ifndef OPENOCD_TARGET_DSP5680XX_H
#define OPENOCD_TARGET_DSP5680XX_H

#include <jtag/jtag.h>

/**
 * @file   dsp5680xx.h
 * @author Rodrigo Rosa <rodrigorosa.LG@gmail.com>
 * @date   Thu Jun  9 18:54:38 2011
 *
 * @brief  Basic support for the 5680xx DSP from Freescale.
 * The chip has two taps in the JTAG chain, the Master tap and the Core tap.
 * In this code the Master tap is only used to unlock the flash memory by executing a JTAG instruction.
 *
 */

#define S_FILE_DATA_OFFSET 0x200000
#define TIME_DIV_FREESCALE 0.3

/** ----------------------------------------------------------------
 * JTAG
 *----------------------------------------------------------------
 */
#define DSP5680XX_JTAG_CORE_TAP_IRLEN	4
#define DSP5680XX_JTAG_MASTER_TAP_IRLEN	8

#define JTAG_STATUS_MASK		0x0F

#define JTAG_STATUS_NORMAL		0x01
#define JTAG_STATUS_STOPWAIT		0x05
#define JTAG_STATUS_BUSY		0x09
#define JTAG_STATUS_DEBUG		0x0D
#define JTAG_STATUS_DEAD		0x0f

#define JTAG_INSTR_EXTEST		0x0
#define JTAG_INSTR_SAMPLE_PRELOAD	0x1
#define JTAG_INSTR_IDCODE		0x2
#define JTAG_INSTR_EXTEST_PULLUP	0x3
#define JTAG_INSTR_HIGHZ		0x4
#define JTAG_INSTR_CLAMP		0x5
#define JTAG_INSTR_ENABLE_ONCE		0x6
#define JTAG_INSTR_DEBUG_REQUEST	0x7
#define JTAG_INSTR_BYPASS		0xF
 /**
 * ----------------------------------------------------------------
 */

/** ----------------------------------------------------------------
 * Master TAP instructions from MC56F8000RM.pdf
 * ----------------------------------------------------------------
 */
#define MASTER_TAP_CMD_BYPASS      0xF
#define MASTER_TAP_CMD_IDCODE      0x2
#define MASTER_TAP_CMD_TLM_SEL     0x5
#define MASTER_TAP_CMD_FLASH_ERASE 0x8
 /**
 * ----------------------------------------------------------------
 */

 /** ----------------------------------------------------------------
 * EOnCE control register info
 * ----------------------------------------------------------------
 */
#define DSP5680XX_ONCE_OCR_EX	(1<<5)
/* EX Bit Definition
   0 Remain in the Debug Processing State
   1 Leave the Debug Processing State */
#define DSP5680XX_ONCE_OCR_GO	(1<<6)
/* GO Bit Definition
   0 Inactive—No Action Taken
   1 Execute Controller Instruction */
#define DSP5680XX_ONCE_OCR_RW	(1<<7)
/** RW Bit Definition
 * 0 Write To the Register Specified by the RS[4:0] Bits
 * 1 ReadFrom the Register Specified by the RS[4:0] Bits
 * ----------------------------------------------------------------
 */

 /** ----------------------------------------------------------------
 * EOnCE Status Register
 * ----------------------------------------------------------------
 */
#define DSP5680XX_ONCE_OSCR_OS1	 (1<<5)
#define DSP5680XX_ONCE_OSCR_OS0	 (1<<4)
 /**
 * ----------------------------------------------------------------
 */

 /** ----------------------------------------------------------------
 * EOnCE Core Status - Describes the operating status of the core controller
 * ----------------------------------------------------------------
 */
#define DSP5680XX_ONCE_OSCR_NORMAL_M	(0)
/* 00  -   Normal      -   Controller Core Executing Instructions or in Reset */
#define DSP5680XX_ONCE_OSCR_STOPWAIT_M	(DSP5680XX_ONCE_OSCR_OS0)
/* 01  -   Stop/Wait   -   Controller Core in Stop or Wait Mode */
#define DSP5680XX_ONCE_OSCR_BUSY_M	(DSP5680XX_ONCE_OSCR_OS1)
/* 10  -   Busy	-   Controller is Performing External or Peripheral Access (Wait States) */
#define DSP5680XX_ONCE_OSCR_DEBUG_M	(DSP5680XX_ONCE_OSCR_OS0|DSP5680XX_ONCE_OSCR_OS1)
/* 11  -   Debug       -   Controller Core Halted and in Debug Mode */
#define EONCE_STAT_MASK 0x30
 /**
 * ----------------------------------------------------------------
 */

 /** ----------------------------------------------------------------
 * Register Select Encoding (eonce_rev.1.0_0208081.pdf:14)
 * ----------------------------------------------------------------
 */
#define DSP5680XX_ONCE_NOREG    0x00    /* No register selected */
#define DSP5680XX_ONCE_OCR      0x01    /* OnCE Debug Control Register */
#define DSP5680XX_ONCE_OCNTR    0x02    /* OnCE Breakpoint and Trace Counter */
#define DSP5680XX_ONCE_OSR      0x03    /* EOnCE status register */
#define DSP5680XX_ONCE_OBAR     0x04    /* OnCE Breakpoint Address Register */
#define DSP5680XX_ONCE_OBASE    0x05    /* EOnCE Peripheral Base Address register */
#define DSP5680XX_ONCE_OTXRXSR  0x06    /* EOnCE TXRX Status and Control Register (OTXRXSR)  */
#define DSP5680XX_ONCE_OTX      0x07    /* EOnCE Transmit register (OTX)  */
#define DSP5680XX_ONCE_OPDBR    0x08    /* EOnCE Program Data Bus Register (OPDBR) */
#define DSP5680XX_ONCE_OTX1     0x09    /* EOnCE Upper Transmit register (OTX1) */
#define DSP5680XX_ONCE_OPABFR   0x0A    /* OnCE Program Address Register—Fetch cycle */
#define DSP5680XX_ONCE_ORX      0x0B    /* EOnCE Receive register (ORX) */
#define DSP5680XX_ONCE_OCNTR_C  0x0C    /* Clear OCNTR */
#define DSP5680XX_ONCE_ORX1     0x0D    /* EOnCE Upper Receive register (ORX1) */
#define DSP5680XX_ONCE_OTBCR    0x0E    /* EOnCE Trace Buffer Control Reg (OTBCR) */
#define DSP5680XX_ONCE_OPABER   0x10    /* OnCE Program Address Register—Execute cycle */
#define DSP5680XX_ONCE_OPFIFO   0x11    /* OnCE Program address FIFO */
#define DSP5680XX_ONCE_OBAR1    0x12    /* EOnCE Breakpoint 1 Unit 0 Address Reg.(OBAR1) */
#define DSP5680XX_ONCE_OPABDR   0x13    /* OnCE Program Address Register—Decode cycle (OPABDR) */
 /**
 * ----------------------------------------------------------------
 */

#define FLUSH_COUNT_READ_WRITE 8192 /* This value works, higher values (and lower...) may work as well. */
#define FLUSH_COUNT_FLASH 8192
/** ----------------------------------------------------------------
 * HFM (flash module) Commands (ref:MC56F801xRM.pdf:159)
 * ----------------------------------------------------------------
 */
#define HFM_ERASE_VERIFY		  0x05
#define HFM_CALCULATE_DATA_SIGNATURE      0x06
#define HFM_WORD_PROGRAM		  0x20
#define HFM_PAGE_ERASE		    0x40
#define HFM_MASS_ERASE		    0x41
#define HFM_CALCULATE_IFR_BLOCK_SIGNATURE 0x66
 /**
 * ----------------------------------------------------------------
 */

/** ----------------------------------------------------------------
 * Flashing (ref:MC56F801xRM.pdf:159)
 * ----------------------------------------------------------------
 */
#define HFM_BASE_ADDR     0x0F400   /** In x: mem. (write to S_FILE_DATA_OFFSET+HFM_BASE_ADDR
				     * to get data into x: mem.)
				     */
/**
 * The following are register addresses, not memory
 * addresses (though all registers are memory mapped)
 */
#define HFM_CLK_DIV       0x00  /* r/w */
#define HFM_CNFG	  0x01  /* r/w */
#define HFM_SECHI	 0x03  /* r */
#define HFM_SECLO	 0x04  /* r */
#define HFM_PROT	  0x10  /* r/w */
#define HFM_PROTB	 0x11  /* r/w */
#define HFM_USTAT	 0x13  /* r/w */
#define HFM_CMD	   0x14  /* r/w */
#define HFM_DATA	  0x18  /* r */
#define HFM_OPT1	  0x1B  /* r */
#define HFM_TSTSIG	0x1D  /* r */

#define HFM_EXEC_COMPLETE  0x40

/* User status register (USTAT) masks (MC56F80XXRM.pdf:6.7.5) */
#define HFM_USTAT_MASK_BLANK 0x4
#define HFM_USTAT_MASK_PVIOL_ACCER 0x30

/**
 * The value used on for the FM clock is important to prevent flashing errors and to prevent deterioration of the FM.
 * This value was calculated using a spreadsheet tool available on the Freescale website under FAQ 25464.
 *
 */
#define HFM_CLK_DEFAULT	0x27
/* 0x27 according to freescale cfg, but 0x40 according to freescale spreadsheet... */
#define HFM_FLASH_BASE_ADDR 0x0
#define HFM_SIZE_BYTES 0x4000   /* bytes */
#define HFM_SIZE_WORDS 0x2000   /* words */
#define HFM_SECTOR_SIZE 0x200   /* Size in bytes */
#define HFM_SECTOR_COUNT 0x20
/* A 16K block in pages of 256 words. */

/**
 * Writing HFM_LOCK_FLASH to HFM_LOCK_ADDR_L and HFM_LOCK_ADDR_H will enable security on flash after the next reset.
 */
#define HFM_LOCK_FLASH 0xE70A
#define HFM_LOCK_ADDR_L 0x1FF7
#define HFM_LOCK_ADDR_H 0x1FF8
 /**
 * ----------------------------------------------------------------
 */

/** ----------------------------------------------------------------
 * Register Memory Map (eonce_rev.1.0_0208081.pdf:16)
 * ----------------------------------------------------------------
 */
#define MC568013_EONCE_OBASE_ADDR 0xFF
/* The following are relative to EONCE_OBASE_ADDR (EONCE_OBASE_ADDR<<16 + ...) */
#define MC568013_EONCE_TX_RX_ADDR    0xFFFE
#define MC568013_EONCE_TX1_RX1_HIGH_ADDR  0xFFFF /* Relative to EONCE_OBASE_ADDR */
#define MC568013_EONCE_OCR 0xFFA0   /* Relative to EONCE_OBASE_ADDR */
 /**
 * ----------------------------------------------------------------
 */

/** ----------------------------------------------------------------
 * SIM addresses & commands (MC56F80xx.h from freescale)
 * ----------------------------------------------------------------
 */
#define MC568013_SIM_BASE_ADDR 0xF140
#define MC56803X_2X_SIM_BASE_ADDR 0xF100

#define SIM_CMD_RESET 0x10
 /**
 * ----------------------------------------------------------------
 */

/**
 * ----------------------------------------------------------------
 * ERROR codes - enable automatic parsing of output
 * ----------------------------------------------------------------
 */
#define DSP5680XX_ERROR_UNKNOWN_OR_ERROR_OPENOCD -100
#define DSP5680XX_ERROR_JTAG_COMM -1
#define DSP5680XX_ERROR_JTAG_RESET -2
#define DSP5680XX_ERROR_JTAG_INVALID_TAP -3
#define DSP5680XX_ERROR_JTAG_DR_LEN_OVERFLOW -4
#define DSP5680XX_ERROR_INVALID_IR_LEN -5
#define DSP5680XX_ERROR_JTAG_TAP_ENABLE_MASTER -6
#define DSP5680XX_ERROR_JTAG_TAP_ENABLE_CORE -7
#define DSP5680XX_ERROR_JTAG_TAP_FIND_MASTER -8
#define DSP5680XX_ERROR_JTAG_TAP_FIND_CORE -9
#define DSP5680XX_ERROR_JTAG_DRSCAN -10
#define DSP5680XX_ERROR_JTAG_IRSCAN -11
#define DSP5680XX_ERROR_ENTER_DEBUG_MODE -12
#define DSP5680XX_ERROR_RESUME -13
#define DSP5680XX_ERROR_WRITE_WITH_TARGET_RUNNING -14
#define DSP5680XX_ERROR_INVALID_DATA_SIZE_UNIT -15
#define DSP5680XX_ERROR_PROTECT_CHECK_INVALID_ARGS -16
#define DSP5680XX_ERROR_FM_BUSY -17
#define DSP5680XX_ERROR_FM_CMD_TIMED_OUT -18
#define DSP5680XX_ERROR_FM_EXEC -19
#define DSP5680XX_ERROR_FM_SET_CLK -20
#define DSP5680XX_ERROR_FLASHING_INVALID_WORD_COUNT -21
#define DSP5680XX_ERROR_FLASHING_CRC -22
#define DSP5680XX_ERROR_FLASHING -23
#define DSP5680XX_ERROR_NOT_IMPLEMENTED_STEP -24
#define DSP5680XX_ERROR_HALT -25
#define DSP5680XX_ERROR_EXIT_DEBUG_MODE -26
#define DSP5680XX_ERROR_TARGET_RUNNING -27
#define DSP5680XX_ERROR_NOT_IN_DEBUG -28
/**
 * ----------------------------------------------------------------
 */

struct dsp5680xx_common {
	uint32_t stored_pc;
	int flush;
	bool debug_mode_enabled;
};

extern struct dsp5680xx_common dsp5680xx_context;

static inline struct dsp5680xx_common *target_to_dsp5680xx(struct target
							   *target)
{
	return target->arch_info;
}

/**
 * Writes to flash memory.
 * Does not check if flash is erased, it's up to the user to erase the flash before running
 * this function.
 * The flashing algorithm runs from RAM, reading from a register to which this function
 * writes to. The algorithm is open loop, there is no control to verify that the FM read
 * the register before writing the next data. A closed loop approach was much slower,
 * and the current implementation does not fail, and if it did the crc check would detect it,
 * allowing to flash again.
 *
 * @param target
 * @param buffer
 * @param address Word addressing.
 * @param count In bytes.
 * @param is_flash_lock
 *
 * @return
 */
int dsp5680xx_f_wr(struct target *target, const uint8_t *buffer, uint32_t address,
		uint32_t count, int is_flash_lock);

/**
 * The FM has the functionality of checking if the flash array is erased. This function
 * executes it. It does not support individual sector analysis.
 *
 * @param target
 * @param erased
 * @param sector This parameter is ignored because the FM does not support checking if
 * individual sectors are erased.
 *
 * @return
 */
int dsp5680xx_f_erase_check(struct target *target, uint8_t *erased,
		uint32_t sector);

/**
 * Erases either a sector or the complete flash array. If either the range first-last covers
 * the complete array or if first == 0 and last == 0 then a mass erase command is executed
 * on the FM. If not, then individual sectors are erased.
 *
 * @param target
 * @param first
 * @param last
 *
 * @return
 */
int dsp5680xx_f_erase(struct target *target, int first, int last);

/**
 * Reads the memory mapped protection register. A 1 implies the sector is protected,
 * a 0 implies the sector is not protected.
 *
 * @param target
 * @param protected Data read from the protection register.
 *
 * @return
 */
int dsp5680xx_f_protect_check(struct target *target, uint16_t *protected);

/**
 * Writes the flash security words with a specific value. The chip's security will be
 * enabled after the first reset following the execution of this function.
 *
 * @param target
 *
 * @return
 */
int dsp5680xx_f_lock(struct target *target);

/**
 * Executes a mass erase command. The must be done from the Master tap.
 * It is up to the user to select the master tap (jtag tapenable dsp5680xx.chp)
 * before running this function.
 * The flash array will be unsecured (and erased) after the first reset following
 * the execution of this function.
 *
 * @param target
 *
 * @return
 */
int dsp5680xx_f_unlock(struct target *target);

#endif /* OPENOCD_TARGET_DSP5680XX_H */
