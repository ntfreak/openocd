/***************************************************************************
 *   Copyright (C) 2018 by Texas Instruments, Inc.                         *
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

#ifndef OPENOCD_FLASH_NOR_MSP432_H
#define OPENOCD_FLASH_NOR_MSP432_H

/* MSP432 family types */
#define MSP432_NO_FAMILY 0 /* Family type not determined yet */
#define MSP432E4         1 /* MSP432E4 family of devices */
#define MSP432P4         2 /* MSP432P4 family of devices */

/* MSP432 device types */
#define MSP432_NO_TYPE    0 /* Device type not determined yet */
#define MSP432P401X_DEPR  1 /* Early MSP432P401x offerings, now deprecated */
#define MSP432P401X       2 /* MSP432P401x device, revision C or higher */
#define MSP432P411X       3 /* MSP432P411x device, revision A or higher */
#define MSP432P401X_GUESS 4 /* Assuming it's an MSP432P401x device */
#define MSP432P411X_GUESS 5 /* Assuming it's an MSP432P411x device */
#define MSP432E401Y       6 /* MSP432E401Y device */
#define MSP432E411Y       7 /* MSP432E401Y device */
#define MSP432E4X_GUESS   8 /* Assuming it's an MSP432E4x device */

/* Common MSP432 flash parameters */
#define FLASH_BASE 0x00000000

/* MSP432P4 flash parameters */
#define P4_FLASH_MAIN_BASE FLASH_BASE
#define P4_FLASH_INFO_BASE 0x00200000
#define P4_SECTOR_LENGTH   0x1000
#define P4_ALGO_ENTRY_ADDR 0x01000110

/* MSP432E4 flash paramters */
#define E4_FLASH_BASE      FLASH_BASE
#define E4_FLASH_SIZE      0x100000
#define E4_SECTOR_LENGTH   0x4000
#define E4_ALGO_ENTRY_ADDR 0x20000110

/* Flash helper algorithm key addresses */
#define ALGO_BASE_ADDR           0x20000000
#define ALGO_BUFFER1_ADDR        0x20002000
#define ALGO_BUFFER2_ADDR        0x20003000
#define ALGO_PARAMS_BASE_ADDR    0x20000150
#define ALGO_FLASH_COMMAND_ADDR  0x20000150
#define ALGO_RETURN_CODE_ADDR    0x20000154
#define ALGO_FLASH_DEST_ADDR     0x2000015c
#define ALGO_FLASH_LENGTH_ADDR   0x20000160
#define ALGO_BUFFER1_STATUS_ADDR 0x20000164
#define ALGO_BUFFER2_STATUS_ADDR 0x20000168
#define ALGO_ERASE_PARAM_ADDR    0x2000016c
#define ALGO_UNLOCK_BSL_ADDR     0x20000170
#define ALGO_STACK_POINTER_ADDR  0x20002000

/* Flash helper algorithm key sizes */
#define ALGO_BUFFER_SIZE  0x1000
#define ALGO_WORKING_SIZE (ALGO_BUFFER2_ADDR + 0x1000 - ALGO_BASE_ADDR)

/* Flash helper algorithm flash commands */
#define FLASH_NO_COMMAND    0
#define FLASH_MASS_ERASE    1
#define FLASH_SECTOR_ERASE  2
#define FLASH_PROGRAM       4
#define FLASH_INIT          8
#define FLASH_EXIT          16
#define FLASH_CONTINUOUS    32

/* Flash helper algorithm return codes */
#define FLASH_BUSY          0x00000001
#define FLASH_SUCCESS       0x00000ACE
#define FLASH_ERROR         0x0000DEAD
#define FLASH_TIMEOUT_ERROR 0xDEAD0000
#define FLASH_VERIFY_ERROR  0xDEADDEAD
#define FLASH_WRONG_COMMAND 0x00000BAD
#define FLASH_POWER_ERROR   0x00DEAD00

/* Flash helper algorithm buffer status values */
#define BUFFER_INACTIVE   0x00
#define BUFFER_ACTIVE     0x01
#define BUFFER_DATA_READY 0x10

/* Flash helper algorithm erase parameters */
#define FLASH_ERASE_MAIN 0x01
#define FLASH_ERASE_INFO 0x02

/* Flash helper algorithm lock/unlock BSL options */
#define FLASH_LOCK_BSL   0x00
#define FLASH_UNLOCK_BSL 0x0b

/* Flash helper algorithm parameter block struct */
struct msp432_algo_params {
	uint8_t flash_command[4];
	uint8_t return_code[4];
	uint8_t _reserved0[4];
	uint8_t address[4];
	uint8_t length[4];
	uint8_t buffer1_status[4];
	uint8_t buffer2_status[4];
	uint8_t erase_param[4];
	uint8_t unlock_bsl[4];
};

/* Flash helper algorithm for MSP432P401x targets */
const uint8_t msp432p401x_algo[] = {
#include "../../../contrib/loaders/flash/msp432/msp432p401x_algo.inc"
};

/* Flash helper algorithm for MSP432P411x targets */
const uint8_t msp432p411x_algo[] = {
#include "../../../contrib/loaders/flash/msp432/msp432p411x_algo.inc"
};

/* Flash helper algorithm for MSP432E4x targets */
const uint8_t msp432e4x_algo[] = {
#include "../../../contrib/loaders/flash/msp432/msp432e4x_algo.inc"
};

#endif /* OPENOCD_FLASH_NOR_MSP432_H */
