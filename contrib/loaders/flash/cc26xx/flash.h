/******************************************************************************
*
* Copyright (C) 2016-2018 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#ifndef OPENOCD_LOADERS_FLASH_CC26XX_FLASH_H
#define OPENOCD_LOADERS_FLASH_CC26XX_FLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "hw_regs.h"

/* Location of flash in memory map */
#define FLASHMEM_BASE 0

/* Defines to access flash API calls in ROM */
#define ROM_API_TABLE       ((uint32_t *) 0x10000180)
#define ROM_VERSION         (ROM_API_TABLE[0])
#define ROM_API_FLASH_TABLE ((uint32_t *) (ROM_API_TABLE[10]))

#if defined(DEVICE_CC26X2)

/* Agama (CC26x2) specific definitions */

#define FLASH_ERASE_SIZE       8192
/* Agama (and Agama 1M) has a maximum of 132 flash sectors (1056KB / 8KB) */
#define FLASH_MAX_SECTOR_COUNT 132
#define FLASH_SECTOR_BASE_M    0xFFFFE000

/* Bootloader Configuration */
#define CCFG_O_BL_CONFIG 0x00001FD8

#elif defined(DEVICE_CC26X0)

/* Chameleon (CC26x0) specific definitions */

#define FLASH_ERASE_SIZE       4096
/* Chameleon has a maximum of 32 flash sectors (128KB / 4KB) */
#define FLASH_MAX_SECTOR_COUNT 32
#define FLASH_SECTOR_BASE_M    0xFFFFF000

/* Bootloader Configuration */
#define CCFG_O_BL_CONFIG 0x00000FD8

#else
#error No DEVICE defined.
#endif

/******************************************************************************
*
* Values that can be returned from the API functions
*
******************************************************************************/
#define FAPI_STATUS_SUCCESS   0x00000000 /* Function completed successfully */
#define FAPI_STATUS_FSM_BUSY  0x00000001 /* FSM is Busy */
#define FAPI_STATUS_FSM_READY 0x00000002 /* FSM is Ready */
#define FAPI_STATUS_INCORRECT_DATABUFFER_LENGTH \
							  0x00000003 /* Incorrect parameter value */
#define FAPI_STATUS_FSM_ERROR 0x00000004 /* Operation failed */

/******************************************************************************
*
* Define used by the flash programming and erase functions
*
******************************************************************************/
#define ADDR_OFFSET (0x1F800000 - FLASHMEM_BASE)

/******************************************************************************
*
* Define used for access to factory configuration area.
*
******************************************************************************/
#define FCFG1_OFFSET 0x1000

/******************************************************************************
*
* Define for the clock frequency input to the flash module in number of MHz
*
******************************************************************************/
#define FLASH_MODULE_CLK_FREQ 48

/******************************************************************************
*
* Defined values for Flash State Machine commands
*
******************************************************************************/
typedef enum {
	FAPI_PROGRAM_DATA    = 0x0002, /* Program data. */
	FAPI_ERASE_SECTOR    = 0x0006, /* Erase sector. */
	FAPI_ERASE_BANK      = 0x0008, /* Erase bank. */
	FAPI_VALIDATE_SECTOR = 0x000E, /* Validate sector. */
	FAPI_CLEAR_STATUS    = 0x0010, /* Clear status. */
	FAPI_PROGRAM_RESUME  = 0x0014, /* Program resume. */
	FAPI_ERASE_RESUME    = 0x0016, /* Erase resume. */
	FAPI_CLEAR_MORE      = 0x0018, /* Clear more. */
	FAPI_PROGRAM_SECTOR  = 0x0020, /* Program sector. */
	FAPI_ERASE_OTP       = 0x0030  /* Erase OTP. */
} flash_state_command_t;

/******************************************************************************
*
* Defines for values written to the FLASH_O_FSM_WR_ENA register
*
******************************************************************************/
#define FSM_REG_WRT_ENABLE  5
#define FSM_REG_WRT_DISABLE 2

/******************************************************************************
*
* Defines for the bank power mode field the FLASH_O_FBFALLBACK register
*
******************************************************************************/
#define FBFALLBACK_SLEEP      0
#define FBFALLBACK_DEEP_STDBY 1
#define FBFALLBACK_ACTIVE     3

/******************************************************************************
*
* Defines for the bank grace period and pump grace period
*
******************************************************************************/
#define FLASH_BAGP 0x14
#define FLASH_PAGP 0x14

/******************************************************************************
*
* Defines for the FW flag bits in the FLASH_O_FWFLAG register
*
******************************************************************************/
#define FW_WRT_TRIMMED 0x00000001

/******************************************************************************
*
* Defines used by the flash programming functions
*
******************************************************************************/
typedef volatile uint8_t fwp_write_byte;
#define FWPWRITE_BYTE_ADDRESS \
	((fwp_write_byte *)((FLASH_BASE + FLASH_O_FWPWRITE0)))

/******************************************************************************
*
* Define for FSM command execution
*
******************************************************************************/
#define FLASH_CMD_EXEC 0x15

/******************************************************************************
*
* Get size of a flash sector in number of bytes.
*
* This function will return the size of a flash sector in number of bytes.
*
* Returns size of a flash sector in number of bytes.
*
******************************************************************************/
static inline uint32_t flash_sector_size_get(void)
{
	uint32_t sector_size_in_kbyte;

	sector_size_in_kbyte = (HWREG(FLASH_BASE + FLASH_O_FCFG_B0_SSIZE0) &
		FLASH_FCFG_B0_SSIZE0_B0_SECT_SIZE_M) >>
		FLASH_FCFG_B0_SSIZE0_B0_SECT_SIZE_S;

	/* Return flash sector size in number of bytes. */
	return sector_size_in_kbyte * 1024;
}

/******************************************************************************
*
* Get the size of the flash.
*
* This function returns the size of the flash main bank in number of bytes.
*
* Returns the flash size in number of bytes.
*
******************************************************************************/
static inline uint32_t flash_size_get(void)
{
	uint32_t num_of_sectors;

	/* Get number of flash sectors */
	num_of_sectors = (HWREG(FLASH_BASE + FLASH_O_FLASH_SIZE) &
						FLASH_FLASH_SIZE_SECTORS_M) >>
						FLASH_FLASH_SIZE_SECTORS_S;

	/* Return flash size in number of bytes */
	return num_of_sectors * flash_sector_size_get();
}

/******************************************************************************
*
* Checks if the Flash state machine has detected an error.
*
* This function returns the status of the Flash State Machine indicating if
* an error is detected or not. Primary use is to check if an Erase or
* Program operation has failed.
*
* Please note that code can not execute in flash while any part of the flash
* is being programmed or erased. This function must be called from ROM or
* SRAM while any part of the flash is being programmed or erased.
*
* Returns status of Flash state machine:
* FAPI_STATUS_FSM_ERROR
* FAPI_STATUS_SUCCESS
*
******************************************************************************/
static inline uint32_t flash_check_fsm_for_error(void)
{
	if (HWREG(FLASH_BASE + FLASH_O_FMSTAT) & FLASH_FMSTAT_CSTAT)
		return FAPI_STATUS_FSM_ERROR;
	else
		return FAPI_STATUS_SUCCESS;
}

/******************************************************************************
*
* Checks if the Flash state machine is ready.
*
* This function returns the status of the Flash State Machine indicating if
* it is ready to accept a new command or not. Primary use is to check if an
* Erase or Program operation has finished.
*
* Please note that code can not execute in flash while any part of the flash
* is being programmed or erased. This function must be called from ROM or
* SRAM while any part of the flash is being programmed or erased.
*
* Returns readiness status of Flash state machine:
* FAPI_STATUS_FSM_READY
* FAPI_STATUS_FSM_BUSY
*
******************************************************************************/
static inline uint32_t flash_check_fsm_for_ready(void)
{
	if (HWREG(FLASH_BASE + FLASH_O_STAT) & FLASH_STAT_BUSY)
		return FAPI_STATUS_FSM_BUSY;
	else
		return FAPI_STATUS_FSM_READY;
}

/******************************************************************************
*
* Erase a flash sector.
*
* This function will erase the specified flash sector. The function will
* not return until the flash sector has been erased or an error condition
* occurred. If flash top sector is erased the function will program the
* device security data bytes with default values. The device security
* data located in the customer configuration area of the flash top sector,
* must have valid values at all times. These values affect the configuration
* of the device during boot.
*
* Please note that code can not execute in flash while any part of the flash
* is being programmed or erased. This function must only be executed from ROM
* or SRAM.
*
* sector_address is the starting address in flash of the sector to be
* erased.
*
* Returns the status of the sector erase:
* FAPI_STATUS_SUCCESS                     : Success.
* FAPI_STATUS_INCORRECT_DATABUFFER_LENGTH : Invalid argument.
* FAPI_STATUS_FSM_ERROR                   : Programming error was encountered.
*
******************************************************************************/
extern uint32_t flash_sector_erase(uint32_t sector_address);

/******************************************************************************
*
* Erase all unprotected sectors in the flash main bank.
*
* This function will erase all unprotected flash sectors. The function will
* not return until the flash sectors has been erased or an error condition
* occurred. Since the flash top sector is erased the function will program
* the device security data bytes with default values. The device security
* data located in the customer configuration area of the flash top sector,
* must have valid values at all times. These values affect the configuration
* of the device during boot. The execution time of the operation increases if
* erase precondition is forced. This will cause the flash module to first
* program all 1 bits in the bank to 0 before the actual erase is started.
*
* force_precondition controls if erase precondition should be forced.
*
* Returns the status of the sector erase:
* FAPI_STATUS_SUCCESS   : Success
* FAPI_STATUS_FSM_ERROR : Erase error was encountered.
*
******************************************************************************/
extern uint32_t flash_bank_erase(bool force_precondition);

/******************************************************************************
*
* Programs unprotected main bank flash sectors.
*
* This function will program a sequence of bytes into the on-chip flash.
* Programming each location consists of the result of an AND operation
* of the new data and the existing data; in other words bits that contain
* 1 can remain 1 or be changed to 0, but bits that are 0 cannot be changed
* to 1.  Therefore, a byte can be programmed multiple times as long as these
* rules are followed; if a program operation attempts to change a 0 bit to
* a 1 bit, that bit will not have its value changed.
*
* This function will not return until the data has been programmed or an
* programming error has occurred.
*
* Please note that code can not execute in flash while any part of the flash
* is being programmed or erased. This function must only be executed from ROM
* or SRAM.
*
* The data_buffer pointer cannot point to flash.
*
* data_buffer is a pointer to the data to be programmed.
* address is the starting address in flash to be programmed.
* count is the number of bytes to be programmed.
*
* Returns status of the flash programming:
* FAPI_STATUS_SUCCESS                     : Success.
* FAPI_STATUS_INCORRECT_DATABUFFER_LENGTH : Too many bytes were requested.
* FAPI_STATUS_FSM_ERROR                   : Programming error was encountered.
*
******************************************************************************/
extern uint32_t flash_program(uint8_t *data_buffer, uint32_t address,
	uint32_t count);

/******************************************************************************
*
* Disables all sectors for erase and programming on the active bank.
*
* This function disables all sectors for erase and programming on the active
* bank and enables the Idle Reading Power reduction mode if no low power
* mode is configured. Furthermore, an additional level of protection from
* erase is enabled.
*
* Please note that code can not execute in flash while any part of the flash
* is being programmed or erased.
*
******************************************************************************/
extern void flash_disable_sectors_for_write(void);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef OPENOCD_LOADERS_FLASH_CC26XX_FLASH_H */
