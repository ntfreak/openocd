/***************************************************************************
 *   Copyright (C) 2011 by Marc Willam, Holger Wech                        *
 *       openOCD.fseu(AT)de.fujitsu.com                                    *
 *   Copyright (C) 2011 Ronny Strutz                                       *
 *                                                                         *
 *   Copyright (C) 2013 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
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
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

#define FLASH_DQ6 0x40		/* Data toggle flag bit (TOGG) position */
#define FLASH_DQ5 0x20		/* Time limit exceeding flag bit (TLOV) position */

enum fm3_variant {
	mb9bfxx1,	/* Flash Type '1' */
	mb9bfxx2,
	mb9bfxx3,
	mb9bfxx4,
	mb9bfxx5,
	mb9bfxx6,
	mb9bfxx7,
	mb9bfxx8,

	mb9afxx1,	/* Flash Type '2' */
	mb9afxx2,
	mb9afxx3,
	mb9afxx4,
	mb9afxx5,
	mb9afxx6,
	mb9afxx7,
	mb9afxx8,
};

enum fm3_flash_type {
	fm3_no_flash_type = 0,
	fm3_flash_type1   = 1,
	fm3_flash_type2   = 2
};

struct fm3_flash_bank {
	enum fm3_variant variant;
	enum fm3_flash_type flashtype;
	bool probed;
};

FLASH_BANK_COMMAND_HANDLER(fm3_flash_bank_command)
{
	struct fm3_flash_bank *fm3_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	fm3_info = malloc(sizeof(struct fm3_flash_bank));
	bank->driver_priv = fm3_info;

	/* Flash type '1' */
	if (strcmp(CMD_ARGV[5], "mb9bfxx1.cpu") == 0) {
		fm3_info->variant = mb9bfxx1;
		fm3_info->flashtype = fm3_flash_type1;
	} else if (strcmp(CMD_ARGV[5], "mb9bfxx2.cpu") == 0) {
		fm3_info->variant = mb9bfxx2;
		fm3_info->flashtype = fm3_flash_type1;
	} else if (strcmp(CMD_ARGV[5], "mb9bfxx3.cpu") == 0) {
		fm3_info->variant = mb9bfxx3;
		fm3_info->flashtype = fm3_flash_type1;
	} else if (strcmp(CMD_ARGV[5], "mb9bfxx4.cpu") == 0) {
		fm3_info->variant = mb9bfxx4;
		fm3_info->flashtype = fm3_flash_type1;
	} else if (strcmp(CMD_ARGV[5], "mb9bfxx5.cpu") == 0) {
		fm3_info->variant = mb9bfxx5;
		fm3_info->flashtype = fm3_flash_type1;
	} else if (strcmp(CMD_ARGV[5], "mb9bfxx6.cpu") == 0) {
		fm3_info->variant = mb9bfxx6;
		fm3_info->flashtype = fm3_flash_type1;
	} else if (strcmp(CMD_ARGV[5], "mb9bfxx7.cpu") == 0) {
		fm3_info->variant = mb9bfxx7;
		fm3_info->flashtype = fm3_flash_type1;
	} else if (strcmp(CMD_ARGV[5], "mb9bfxx8.cpu") == 0) {
		fm3_info->variant = mb9bfxx8;
		fm3_info->flashtype = fm3_flash_type1;
	} else if (strcmp(CMD_ARGV[5], "mb9afxx1.cpu") == 0) {	/* Flash type '2' */
		fm3_info->variant = mb9afxx1;
		fm3_info->flashtype = fm3_flash_type2;
	} else if (strcmp(CMD_ARGV[5], "mb9afxx2.cpu") == 0) {
		fm3_info->variant = mb9afxx2;
		fm3_info->flashtype = fm3_flash_type2;
	} else if (strcmp(CMD_ARGV[5], "mb9afxx3.cpu") == 0) {
		fm3_info->variant = mb9afxx3;
		fm3_info->flashtype = fm3_flash_type2;
	} else if (strcmp(CMD_ARGV[5], "mb9afxx4.cpu") == 0) {
		fm3_info->variant = mb9afxx4;
		fm3_info->flashtype = fm3_flash_type2;
	} else if (strcmp(CMD_ARGV[5], "mb9afxx5.cpu") == 0) {
		fm3_info->variant = mb9afxx5;
		fm3_info->flashtype = fm3_flash_type2;
	} else if (strcmp(CMD_ARGV[5], "mb9afxx6.cpu") == 0) {
		fm3_info->variant = mb9afxx6;
		fm3_info->flashtype = fm3_flash_type2;
	} else if (strcmp(CMD_ARGV[5], "mb9afxx7.cpu") == 0) {
		fm3_info->variant = mb9afxx7;
		fm3_info->flashtype = fm3_flash_type2;
	} else if (strcmp(CMD_ARGV[5], "mb9afxx8.cpu") == 0) {
		fm3_info->variant = mb9afxx8;
		fm3_info->flashtype = fm3_flash_type2;
	}

	/* unknown Flash type */
	else {
		LOG_ERROR("unknown fm3 variant: %s", CMD_ARGV[5]);
		free(fm3_info);
		return ERROR_FLASH_BANK_INVALID;
	}

	fm3_info->probed = false;

	return ERROR_OK;
}

/* Data polling algorithm */
static int fm3_busy_wait(struct target *target, uint32_t offset, int timeout_ms)
{
	int retval = ERROR_OK;
	uint8_t state1, state2;
	int ms = 0;

	/* While(1) loop exit via "break" and "return" on error */
	while (1) {
		/* dummy-read - see flash manual */
		retval = target_read_u8(target, offset, &state1);
		if (retval != ERROR_OK)
			return retval;

		/* Data polling 1 */
		retval = target_read_u8(target, offset, &state1);
		if (retval != ERROR_OK)
			return retval;

		/* Data polling 2 */
		retval = target_read_u8(target, offset, &state2);
		if (retval != ERROR_OK)
			return retval;

		/* Flash command finished via polled data equal? */
		if ((state1 & FLASH_DQ6) == (state2 & FLASH_DQ6))
			break;
		/* Timeout Flag? */
		else if (state1 & FLASH_DQ5) {
			/* Retry data polling */

			/* Data polling 1 */
			retval = target_read_u8(target, offset, &state1);
			if (retval != ERROR_OK)
				return retval;

			/* Data polling 2 */
			retval = target_read_u8(target, offset, &state2);
			if (retval != ERROR_OK)
				return retval;

			/* Flash command finished via polled data equal? */
			if ((state1 & FLASH_DQ6) != (state2 & FLASH_DQ6))
				return ERROR_FLASH_OPERATION_FAILED;

			/* finish anyway */
			break;
		}
		usleep(1000);
		++ms;

		/* Polling time exceeded? */
		if (ms > timeout_ms) {
			LOG_ERROR("Polling data reading timed out!");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	if (retval == ERROR_OK)
		LOG_DEBUG("fm3_busy_wait(%" PRIx32 ") needs about %d ms", offset, ms);

	return retval;
}

static int fm3_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct fm3_flash_bank *fm3_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval = ERROR_OK;
	uint32_t u32DummyRead;
	int odd;
	uint32_t u32FlashType;
	uint32_t u32FlashSeqAddress1;
	uint32_t u32FlashSeqAddress2;

	struct working_area *write_algorithm;
	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;

	u32FlashType = (uint32_t) fm3_info->flashtype;

	if (u32FlashType == fm3_flash_type1) {
		u32FlashSeqAddress1 = 0x00001550;
		u32FlashSeqAddress2 = 0x00000AA8;
	} else if (u32FlashType == fm3_flash_type2) {
		u32FlashSeqAddress1 = 0x00000AA8;
		u32FlashSeqAddress2 = 0x00000554;
	} else {
		LOG_ERROR("Flash/Device type unknown!");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* RAMCODE used for fm3 Flash sector erase:				   */
	/* R0 keeps Flash Sequence address 1     (u32FlashSeq1)    */
	/* R1 keeps Flash Sequence address 2     (u32FlashSeq2)    */
	/* R2 keeps Flash Offset address         (ofs)			   */
	static const uint8_t fm3_flash_erase_sector_code[] = {
						/*    *(uint16_t*)u32FlashSeq1 = 0xAA; */
		0xAA, 0x24,		/*        MOVS  R4, #0xAA              */
		0x04, 0x80,		/*        STRH  R4, [R0, #0]           */
						/*    *(uint16_t*)u32FlashSeq2 = 0x55; */
		0x55, 0x23,		/*        MOVS  R3, #0x55              */
		0x0B, 0x80,		/*        STRH  R3, [R1, #0]           */
						/*    *(uint16_t*)u32FlashSeq1 = 0x80; */
		0x80, 0x25,		/*        MOVS  R5, #0x80              */
		0x05, 0x80,		/*        STRH  R5, [R0, #0]           */
						/*    *(uint16_t*)u32FlashSeq1 = 0xAA; */
		0x04, 0x80,		/*        STRH  R4, [R0, #0]           */
						/*    *(uint16_t*)u32FlashSeq2 = 0x55; */
		0x0B, 0x80,		/*        STRH  R3, [R1, #0]           */
						/* Sector_Erase Command (0x30)         */
						/*    *(uint16_t*)ofs = 0x30;          */
		0x30, 0x20,		/*        MOVS  R0, #0x30              */
		0x10, 0x80,		/*        STRH  R0, [R2, #0]           */
						/* End Code                            */
		0x00, 0xBE,		/*        BKPT  #0                     */
	};

	LOG_INFO("Fujitsu MB9[A/B]FXXX: Sector Erase ... (%u to %u)", first, last);

	/* disable HW watchdog */
	retval = target_write_u32(target, 0x40011C00, 0x1ACCE551);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, 0x40011C00, 0xE5331AAE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, 0x40011008, 0x00000000);
	if (retval != ERROR_OK)
		return retval;

	/* FASZR = 0x01, Enables CPU Programming Mode (16-bit Flash access) */
	retval = target_write_u32(target, 0x40000000, 0x0001);
	if (retval != ERROR_OK)
		return retval;

	/* dummy read of FASZR */
	retval = target_read_u32(target, 0x40000000, &u32DummyRead);
	if (retval != ERROR_OK)
		return retval;

	/* allocate working area with flash sector erase code */
	if (target_alloc_working_area(target, sizeof(fm3_flash_erase_sector_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	retval = target_write_buffer(target, write_algorithm->address,
		sizeof(fm3_flash_erase_sector_code), fm3_flash_erase_sector_code);
	if (retval != ERROR_OK)
		return retval;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /* u32FlashSeqAddress1 */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* u32FlashSeqAddress2 */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /* offset				*/

	/* write code buffer and use Flash sector erase code within fm3				*/
	for (unsigned int sector = first ; sector <= last ; sector++) {
		uint32_t offset = bank->sectors[sector].offset;

		for (odd = 0; odd < 2 ; odd++) {
			if (odd)
				offset += 4;

			buf_set_u32(reg_params[0].value, 0, 32, u32FlashSeqAddress1);
			buf_set_u32(reg_params[1].value, 0, 32, u32FlashSeqAddress2);
			buf_set_u32(reg_params[2].value, 0, 32, offset);

			retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
					write_algorithm->address, 0, 100000, &armv7m_info);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error executing flash erase programming algorithm");
				retval = ERROR_FLASH_OPERATION_FAILED;
				return retval;
			}

			retval = fm3_busy_wait(target, offset, 500);
			if (retval != ERROR_OK)
				return retval;
		}
		bank->sectors[sector].is_erased = 1;
	}

	target_free_working_area(target, write_algorithm);
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	/* FASZR = 0x02, Enables CPU Run Mode (32-bit Flash access) */
	retval = target_write_u32(target, 0x40000000, 0x0002);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, 0x40000000, &u32DummyRead); /* dummy read of FASZR */

	return retval;
}

static int fm3_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct fm3_flash_bank *fm3_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 2048;		/* Default minimum value */
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;
	uint32_t u32FlashType;
	uint32_t u32FlashSeqAddress1;
	uint32_t u32FlashSeqAddress2;

	/* Increase buffer_size if needed */
	if (buffer_size < (target->working_area_size / 2))
		buffer_size = (target->working_area_size / 2);

	u32FlashType = (uint32_t) fm3_info->flashtype;

	if (u32FlashType == fm3_flash_type1) {
		u32FlashSeqAddress1 = 0x00001550;
		u32FlashSeqAddress2 = 0x00000AA8;
	} else if (u32FlashType == fm3_flash_type2) {
		u32FlashSeqAddress1 = 0x00000AA8;
		u32FlashSeqAddress2 = 0x00000554;
	} else {
		LOG_ERROR("Flash/Device type unknown!");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* RAMCODE used for fm3 Flash programming:                 */
	/* R0 keeps source start address         (u32Source)       */
	/* R1 keeps target start address         (u32Target)       */
	/* R2 keeps number of halfwords to write (u32Count)        */
	/* R3 keeps Flash Sequence address 1     (u32FlashSeq1)    */
	/* R4 keeps Flash Sequence address 2     (u32FlashSeq2)    */
	/* R5 returns result value               (u32FlashResult)  */

	static const uint8_t fm3_flash_write_code[] = {
								/*    fm3_FLASH_IF->FASZ &= 0xFFFD;           */
	0x5F, 0xF0, 0x80, 0x45,		/*        MOVS.W   R5, #(fm3_FLASH_IF->FASZ)  */
	0x2D, 0x68,					/*        LDR      R5, [R5]                   */
	0x4F, 0xF6, 0xFD, 0x76,		/*        MOVW     R6, #0xFFFD                */
	0x35, 0x40,					/*        ANDS     R5, R5, R6                 */
	0x5F, 0xF0, 0x80, 0x46,		/*        MOVS.W   R6, #(fm3_FLASH_IF->FASZ)  */
	0x35, 0x60,					/*        STR      R5, [R6]                   */
								/*    fm3_FLASH_IF->FASZ |= 1;                */
	0x5F, 0xF0, 0x80, 0x45,		/*        MOVS.W   R5, #(fm3_FLASH_IF->FASZ)  */
	0x2D, 0x68,					/*        LDR      R5, [R3]                   */
	0x55, 0xF0, 0x01, 0x05,		/*        ORRS.W   R5, R5, #1                 */
	0x5F, 0xF0, 0x80, 0x46,		/*        MOVS.W   R6, #(fm3_FLASH_IF->FASZ)  */
	0x35, 0x60,					/*        STR      R5, [R6]                   */
								/*    u32DummyRead = fm3_FLASH_IF->FASZ;      */
	0x28, 0x4D,					/*        LDR.N    R5, ??u32DummyRead         */
	0x5F, 0xF0, 0x80, 0x46,		/*        MOVS.W   R6, #(fm3_FLASH_IF->FASZ)  */
	0x36, 0x68,					/*        LDR      R6, [R6]                   */
	0x2E, 0x60,					/*        STR      R6, [R5]                   */
								/*    u32FlashResult = FLASH_WRITE_NO_RESULT  */
	0x26, 0x4D,					/*        LDR.N    R5, ??u32FlashResult       */
	0x00, 0x26,					/*        MOVS     R6, #0                     */
	0x2E, 0x60,					/*        STR      R6, [R5]                   */
								/*    while ((u32Count > 0 )                  */
								/*      && (u32FlashResult                    */
								/*          == FLASH_WRITE_NO_RESULT))        */
	0x01, 0x2A,					/* L0:    CMP      R2, #1                     */
	0x2C, 0xDB,					/*        BLT.N    L1                         */
	0x24, 0x4D,					/*        LDR.N    R5, ??u32FlashResult       */
	0x2D, 0x68,					/*        LDR      R5, [R5]                   */
	0x00, 0x2D,					/*        CMP      R5, #0                     */
	0x28, 0xD1,					/*        BNE.N    L1                         */
								/*    *u32FlashSeq1 = FLASH_WRITE_1;          */
	0xAA, 0x25,					/*        MOVS     R5, #0xAA                  */
	0x1D, 0x60,					/*        STR      R5, [R3]                   */
								/*    *u32FlashSeq2 = FLASH_WRITE_2;          */
	0x55, 0x25,					/*        MOVS     R5, #0x55                  */
	0x25, 0x60,					/*        STR      R5, [R4]                   */
								/*    *u32FlashSeq1 = FLASH_WRITE_3;          */
	0xA0, 0x25,					/*        MOVS     R5, #0xA0                  */
	0x1D, 0x60,					/*        STRH     R5, [R3]                   */
								/*    *(volatile uint16_t*)u32Target          */
								/*      = *(volatile uint16_t*)u32Source;     */
	0x05, 0x88,					/*        LDRH     R5, [R0]                   */
	0x0D, 0x80,					/*        STRH     R5, [R1]                   */
								/*    while (u32FlashResult                   */
								/*           == FLASH_WRITE_NO_RESTULT)       */
	0x1E, 0x4D,					/* L2:    LDR.N    R5, ??u32FlashResult       */
	0x2D, 0x68,					/*        LDR      R5, [R5]                   */
	0x00, 0x2D,					/*        CMP      R5, #0                     */
	0x11, 0xD1,					/*        BNE.N    L3                         */
								/*    if ((*(volatile uint16_t*)u32Target     */
								/*        & FLASH_DQ5) == FLASH_DQ5)          */
	0x0D, 0x88,					/*        LDRH     R5, [R1]                   */
	0xAD, 0x06,					/*        LSLS     R5, R5, #0x1A              */
	0x02, 0xD5,					/*        BPL.N    L4                         */
								/*    u32FlashResult = FLASH_WRITE_TIMEOUT    */
	0x1A, 0x4D,					/*        LDR.N    R5, ??u32FlashResult       */
	0x02, 0x26,					/*        MOVS     R6, #2                     */
	0x2E, 0x60,					/*        STR      R6, [R5]                   */
								/*    if ((*(volatile uint16_t *)u32Target    */
								/*         & FLASH_DQ7)                       */
								/*        == (*(volatile uint16_t*)u32Source  */
								/*            & FLASH_DQ7))                   */
	0x0D, 0x88,					/* L4:    LDRH     R5, [R1]                   */
	0x15, 0xF0, 0x80, 0x05,		/*        ANDS.W   R5, R5, #0x80              */
	0x06, 0x88,					/*        LDRH     R6, [R0]                   */
	0x16, 0xF0, 0x80, 0x06,		/*        ANDS.W   R6, R6, #0x80              */
	0xB5, 0x42,					/*        CMP      R5, R6                     */
	0xED, 0xD1,					/*        BNE.N    L2                         */
								/*    u32FlashResult = FLASH_WRITE_OKAY       */
	0x15, 0x4D,					/*        LDR.N    R5, ??u32FlashResult       */
	0x01, 0x26,					/*        MOVS     R6, #1                     */
	0x2E, 0x60,					/*        STR      R6, [R5]                   */
	0xE9, 0xE7,					/*        B.N      L2                         */
								/*    if (u32FlashResult                      */
								/*        != FLASH_WRITE_TIMEOUT)             */
	0x13, 0x4D,					/*        LDR.N    R5, ??u32FlashResult       */
	0x2D, 0x68,					/*        LDR      R5, [R5]                   */
	0x02, 0x2D,					/*        CMP      R5, #2                     */
	0x02, 0xD0,					/*        BEQ.N    L5                         */
								/*    u32FlashResult = FLASH_WRITE_NO_RESULT  */
	0x11, 0x4D,					/*        LDR.N    R5, ??u32FlashResult       */
	0x00, 0x26,					/*        MOVS     R6, #0                     */
	0x2E, 0x60,					/*        STR      R6, [R5]                   */
								/*    u32Count--;                             */
	0x52, 0x1E,					/* L5:    SUBS     R2, R2, #1                 */
								/*    u32Source += 2;                         */
	0x80, 0x1C,					/*        ADDS     R0, R0, #2                 */
								/*    u32Target += 2;                         */
	0x89, 0x1C,					/*        ADDS     R1, R1, #2                 */
	0xD0, 0xE7,					/*        B.N      L0                         */
								/*    fm3_FLASH_IF->FASZ &= 0xFFFE;           */
	0x5F, 0xF0, 0x80, 0x45,		/* L1:    MOVS.W   R5, #(fm3_FLASH_IF->FASZ)  */
	0x2D, 0x68,					/*        LDR      R5, [R5]                   */
	0x4F, 0xF6, 0xFE, 0x76,		/*        MOVW     R6, #0xFFFE                */
	0x35, 0x40,					/*        ANDS     R5, R5, R6                 */
	0x5F, 0xF0, 0x80, 0x46,		/*        MOVS.W   R6, #(fm3_FLASH_IF->FASZ)  */
	0x35, 0x60,					/*        STR      R5, [R6]                   */
								/*    fm3_FLASH_IF->FASZ |= 2;                */
	0x5F, 0xF0, 0x80, 0x45,		/*        MOVS.W   R5, #(fm3_FLASH_IF->FASZ)  */
	0x2D, 0x68,					/*        LDR      R5, [R5]                   */
	0x55, 0xF0, 0x02, 0x05,		/*        ORRS.W   R5, R5, #2                 */
	0x5F, 0xF0, 0x80, 0x46,		/*        MOVS.W   R6, #(fm3_FLASH_IF->FASZ)  */
	0x35, 0x60,					/*        STR      R5, [R6]                   */
								/*    u32DummyRead = fm3_FLASH_IF->FASZ;      */
	0x04, 0x4D,					/*        LDR.N    R5, ??u32DummyRead         */
	0x5F, 0xF0, 0x80, 0x46,		/*        MOVS.W   R6, #(fm3_FLASH_IF->FASZ)  */
	0x36, 0x68,					/*        LDR      R6, [R6]                   */
	0x2E, 0x60,					/*        STR      R6, [R5]                   */
								/*    copy u32FlashResult to R3 for return    */
								/*      value                                 */
	0xDF, 0xF8, 0x08, 0x50,		/*        LDR.W    R5, ??u32FlashResult       */
	0x2D, 0x68,					/*        LDR      R5, [R5]                   */
								/*    Breakpoint here                         */
	0x00, 0xBE,					/*        BKPT     #0                         */

	/* The following address pointers assume, that the code is running from   */
	/* SRAM basic-address + 8.These address pointers will be patched, if a    */
	/* different start address in RAM is used (e.g. for Flash type 2)!        */
	/* Default SRAM basic-address is 0x20000000.                              */
	0x00, 0x00, 0x00, 0x20,     /* u32DummyRead address in RAM (0x20000000)   */
	0x04, 0x00, 0x00, 0x20      /* u32FlashResult address in RAM (0x20000004) */
	};

	LOG_INFO("Fujitsu MB9[A/B]FXXX: FLASH Write ...");

	/* disable HW watchdog */
	retval = target_write_u32(target, 0x40011C00, 0x1ACCE551);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, 0x40011C00, 0xE5331AAE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, 0x40011008, 0x00000000);
	if (retval != ERROR_OK)
		return retval;

	count = count / 2;		/* number bytes -> number halfwords */

	/* check code alignment */
	if (offset & 0x1) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* allocate working area and variables with flash programming code */
	if (target_alloc_working_area(target, sizeof(fm3_flash_write_code) + 8,
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address + 8,
		sizeof(fm3_flash_write_code), fm3_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* Patching 'local variable address' */
	/* Algorithm: u32DummyRead: */
	retval = target_write_u32(target, (write_algorithm->address + 8)
			+ sizeof(fm3_flash_write_code) - 8, (write_algorithm->address));
	if (retval != ERROR_OK)
		return retval;
	/* Algorithm: u32FlashResult: */
	retval = target_write_u32(target, (write_algorithm->address + 8)
			+ sizeof(fm3_flash_write_code) - 4, (write_algorithm->address) + 4);
	if (retval != ERROR_OK)
		return retval;



	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* free working area, write algorithm already allocated */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("No large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /* source start address */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* target start address */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT); /* number of halfwords to program */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT); /* Flash Sequence address 1 */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT); /* Flash Sequence address 1 */
	init_reg_param(&reg_params[5], "r5", 32, PARAM_IN);  /* result */

	/* write code buffer and use Flash programming code within fm3           */
	/* Set breakpoint to 0 with time-out of 1000 ms                          */
	while (count > 0) {
		uint32_t thisrun_count = (count > (buffer_size / 2)) ? (buffer_size / 2) : count;

		retval = target_write_buffer(target, source->address, thisrun_count * 2, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);
		buf_set_u32(reg_params[3].value, 0, 32, u32FlashSeqAddress1);
		buf_set_u32(reg_params[4].value, 0, 32, u32FlashSeqAddress2);

		retval = target_run_algorithm(target, 0, NULL, 6, reg_params,
				(write_algorithm->address + 8), 0, 1000, &armv7m_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error executing fm3 Flash programming algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (buf_get_u32(reg_params[5].value, 0, 32) != ERROR_OK) {
			LOG_ERROR("Fujitsu MB9[A/B]FXXX: Flash programming ERROR (Timeout) -> Reg R3: %" PRIx32,
				buf_get_u32(reg_params[5].value, 0, 32));
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer  += thisrun_count * 2;
		address += thisrun_count * 2;
		count   -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

	return retval;
}

static int fm3_probe(struct flash_bank *bank)
{
	struct fm3_flash_bank *fm3_info = bank->driver_priv;
	uint16_t num_pages;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

/*
 -- page-- start -- blocksize - mpu - totalFlash --
	page0 0x00000	16k
	page1 0x04000	16k
	page2 0x08000	96k		___ fxx3  128k Flash
	page3 0x20000  128k		___ fxx4  256k Flash
	page4 0x40000  128k		___ fxx5  384k Flash
	page5 0x60000  128k		___ fxx6  512k Flash
-----------------------
	page6 0x80000  128k
	page7 0xa0000  128k		___ fxx7  256k Flash
	page8 0xc0000  128k
	page9 0xe0000  128k		___ fxx8  256k Flash
 */

	num_pages = 10;				/* max number of Flash pages for malloc */
	fm3_info->probed = false;

	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	bank->base = 0x00000000;
	bank->size = 32 * 1024;		/* bytes */

	bank->sectors[0].offset = 0;
	bank->sectors[0].size = 16 * 1024;
	bank->sectors[0].is_erased = -1;
	bank->sectors[0].is_protected = -1;

	bank->sectors[1].offset = 0x4000;
	bank->sectors[1].size = 16 * 1024;
	bank->sectors[1].is_erased = -1;
	bank->sectors[1].is_protected = -1;

	if ((fm3_info->variant == mb9bfxx1)
	    || (fm3_info->variant == mb9afxx1)) {
		num_pages = 3;
		bank->size = 64 * 1024; /* bytes */
		bank->num_sectors = num_pages;

		bank->sectors[2].offset = 0x8000;
		bank->sectors[2].size = 32 * 1024;
		bank->sectors[2].is_erased = -1;
		bank->sectors[2].is_protected = -1;
	}

	if ((fm3_info->variant == mb9bfxx2)
		|| (fm3_info->variant == mb9bfxx4)
		|| (fm3_info->variant == mb9bfxx5)
		|| (fm3_info->variant == mb9bfxx6)
		|| (fm3_info->variant == mb9bfxx7)
		|| (fm3_info->variant == mb9bfxx8)
		|| (fm3_info->variant == mb9afxx2)
		|| (fm3_info->variant == mb9afxx4)
		|| (fm3_info->variant == mb9afxx5)
		|| (fm3_info->variant == mb9afxx6)
		|| (fm3_info->variant == mb9afxx7)
		|| (fm3_info->variant == mb9afxx8)) {
		num_pages = 3;
		bank->size = 128 * 1024; /* bytes */
		bank->num_sectors = num_pages;

		bank->sectors[2].offset = 0x8000;
		bank->sectors[2].size = 96 * 1024;
		bank->sectors[2].is_erased = -1;
		bank->sectors[2].is_protected = -1;
	}

	if ((fm3_info->variant == mb9bfxx4)
		|| (fm3_info->variant == mb9bfxx5)
		|| (fm3_info->variant == mb9bfxx6)
		|| (fm3_info->variant == mb9bfxx7)
		|| (fm3_info->variant == mb9bfxx8)
		|| (fm3_info->variant == mb9afxx4)
		|| (fm3_info->variant == mb9afxx5)
		|| (fm3_info->variant == mb9afxx6)
		|| (fm3_info->variant == mb9afxx7)
		|| (fm3_info->variant == mb9afxx8)) {
		num_pages = 4;
		bank->size = 256 * 1024; /* bytes */
		bank->num_sectors = num_pages;

		bank->sectors[3].offset = 0x20000;
		bank->sectors[3].size = 128 * 1024;
		bank->sectors[3].is_erased = -1;
		bank->sectors[3].is_protected = -1;
	}

	if ((fm3_info->variant == mb9bfxx5)
		|| (fm3_info->variant == mb9bfxx6)
		|| (fm3_info->variant == mb9bfxx7)
		|| (fm3_info->variant == mb9bfxx8)
		|| (fm3_info->variant == mb9afxx5)
		|| (fm3_info->variant == mb9afxx6)
		|| (fm3_info->variant == mb9afxx7)
		|| (fm3_info->variant == mb9afxx8)) {
		num_pages = 5;
		bank->size = 384 * 1024; /* bytes */
		bank->num_sectors = num_pages;

		bank->sectors[4].offset = 0x40000;
		bank->sectors[4].size = 128 * 1024;
		bank->sectors[4].is_erased = -1;
		bank->sectors[4].is_protected = -1;
	}

	if ((fm3_info->variant == mb9bfxx6)
		|| (fm3_info->variant == mb9bfxx7)
		|| (fm3_info->variant == mb9bfxx8)
		|| (fm3_info->variant == mb9afxx6)
		|| (fm3_info->variant == mb9afxx7)
		|| (fm3_info->variant == mb9afxx8)) {
		num_pages = 6;
		bank->size = 512 * 1024; /* bytes */
		bank->num_sectors = num_pages;

		bank->sectors[5].offset = 0x60000;
		bank->sectors[5].size = 128 * 1024;
		bank->sectors[5].is_erased = -1;
		bank->sectors[5].is_protected = -1;
	}

	if ((fm3_info->variant == mb9bfxx7)
		|| (fm3_info->variant == mb9bfxx8)
		|| (fm3_info->variant == mb9afxx7)
		|| (fm3_info->variant == mb9afxx8)) {
		num_pages = 8;
		bank->size = 768 * 1024; /* bytes */
		bank->num_sectors = num_pages;

		bank->sectors[6].offset = 0x80000;
		bank->sectors[6].size = 128 * 1024;
		bank->sectors[6].is_erased = -1;
		bank->sectors[6].is_protected = -1;

		bank->sectors[7].offset = 0xa0000;
		bank->sectors[7].size = 128 * 1024;
		bank->sectors[7].is_erased = -1;
		bank->sectors[7].is_protected = -1;
	}

	if ((fm3_info->variant == mb9bfxx8)
		|| (fm3_info->variant == mb9afxx8)) {
		num_pages = 10;
		bank->size = 1024 * 1024; /* bytes */
		bank->num_sectors = num_pages;

		bank->sectors[8].offset = 0xc0000;
		bank->sectors[8].size = 128 * 1024;
		bank->sectors[8].is_erased = -1;
		bank->sectors[8].is_protected = -1;

		bank->sectors[9].offset = 0xe0000;
		bank->sectors[9].size = 128 * 1024;
		bank->sectors[9].is_erased = -1;
		bank->sectors[9].is_protected = -1;
	}

	fm3_info->probed = true;

	return ERROR_OK;
}

static int fm3_auto_probe(struct flash_bank *bank)
{
	struct fm3_flash_bank *fm3_info = bank->driver_priv;
	if (fm3_info->probed)
		return ERROR_OK;
	return fm3_probe(bank);
}

/* Chip erase */
static int fm3_chip_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct fm3_flash_bank *fm3_info2 = bank->driver_priv;
	int retval = ERROR_OK;
	uint32_t u32DummyRead;
	uint32_t u32FlashType;
	uint32_t u32FlashSeqAddress1;
	uint32_t u32FlashSeqAddress2;

	struct working_area *write_algorithm;
	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;

	u32FlashType = (uint32_t) fm3_info2->flashtype;

	if (u32FlashType == fm3_flash_type1) {
		LOG_INFO("*** Erasing mb9bfxxx type");
		u32FlashSeqAddress1 = 0x00001550;
		u32FlashSeqAddress2 = 0x00000AA8;
	} else if (u32FlashType == fm3_flash_type2) {
		LOG_INFO("*** Erasing mb9afxxx type");
		u32FlashSeqAddress1 = 0x00000AA8;
		u32FlashSeqAddress2 = 0x00000554;
	} else {
		LOG_ERROR("Flash/Device type unknown!");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* RAMCODE used for fm3 Flash chip erase:				   */
	/* R0 keeps Flash Sequence address 1     (u32FlashSeq1)    */
	/* R1 keeps Flash Sequence address 2     (u32FlashSeq2)    */
	static const uint8_t fm3_flash_erase_chip_code[] = {
						/*    *(uint16_t*)u32FlashSeq1 = 0xAA; */
		0xAA, 0x22,		/*        MOVS  R2, #0xAA              */
		0x02, 0x80,		/*        STRH  R2, [R0, #0]           */
						/*    *(uint16_t*)u32FlashSeq2 = 0x55; */
		0x55, 0x23,		/*        MOVS  R3, #0x55              */
		0x0B, 0x80,		/*        STRH  R3, [R1, #0]           */
						/*    *(uint16_t*)u32FlashSeq1 = 0x80; */
		0x80, 0x24,		/*        MOVS  R4, #0x80              */
		0x04, 0x80,		/*        STRH  R4, [R0, #0]           */
						/*    *(uint16_t*)u32FlashSeq1 = 0xAA; */
		0x02, 0x80,		/*        STRH  R2, [R0, #0]           */
						/*    *(uint16_t*)u32FlashSeq2 = 0x55; */
		0x0B, 0x80,		/*        STRH  R3, [R1, #0]           */
						/* Chip_Erase Command 0x10             */
						/*    *(uint16_t*)u32FlashSeq1 = 0x10; */
		0x10, 0x21,		/*        MOVS  R1, #0x10              */
		0x01, 0x80,		/*        STRH  R1, [R0, #0]           */
						/* End Code                            */
		0x00, 0xBE,		/*        BKPT  #0                      */
	};

	LOG_INFO("Fujitsu MB9[A/B]xxx: Chip Erase ... (may take several seconds)");

	/* disable HW watchdog */
	retval = target_write_u32(target, 0x40011C00, 0x1ACCE551);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, 0x40011C00, 0xE5331AAE);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, 0x40011008, 0x00000000);
	if (retval != ERROR_OK)
		return retval;

	/* FASZR = 0x01, Enables CPU Programming Mode (16-bit Flash access) */
	retval = target_write_u32(target, 0x40000000, 0x0001);
	if (retval != ERROR_OK)
		return retval;

	/* dummy read of FASZR */
	retval = target_read_u32(target, 0x40000000, &u32DummyRead);
	if (retval != ERROR_OK)
		return retval;

	/* allocate working area with flash chip erase code */
	if (target_alloc_working_area(target, sizeof(fm3_flash_erase_chip_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	retval = target_write_buffer(target, write_algorithm->address,
		sizeof(fm3_flash_erase_chip_code), fm3_flash_erase_chip_code);
	if (retval != ERROR_OK)
		return retval;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT); /* u32FlashSeqAddress1 */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT); /* u32FlashSeqAddress2 */

	buf_set_u32(reg_params[0].value, 0, 32, u32FlashSeqAddress1);
	buf_set_u32(reg_params[1].value, 0, 32, u32FlashSeqAddress2);

	retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
			write_algorithm->address, 0, 100000, &armv7m_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error executing flash erase programming algorithm");
		retval = ERROR_FLASH_OPERATION_FAILED;
		return retval;
	}

	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

	retval = fm3_busy_wait(target, u32FlashSeqAddress2, 20000);	/* 20s timeout */
	if (retval != ERROR_OK)
		return retval;

	/* FASZR = 0x02, Re-enables CPU Run Mode (32-bit Flash access) */
	retval = target_write_u32(target, 0x40000000, 0x0002);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, 0x40000000, &u32DummyRead); /* dummy read of FASZR */

	return retval;
}

COMMAND_HANDLER(fm3_handle_chip_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (fm3_chip_erase(bank) == ERROR_OK) {
		/* set all sectors as erased */
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "fm3 chip erase complete");
	} else {
		command_print(CMD, "fm3 chip erase failed");
	}

	return ERROR_OK;
}

static const struct command_registration fm3_exec_command_handlers[] = {
	{
		.name = "chip_erase",
		.usage = "<bank>",
		.handler = fm3_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase entire Flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration fm3_command_handlers[] = {
	{
		.name = "fm3",
		.mode = COMMAND_ANY,
		.help = "fm3 Flash command group",
		.usage = "",
		.chain = fm3_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver fm3_flash = {
	.name = "fm3",
	.commands = fm3_command_handlers,
	.flash_bank_command = fm3_flash_bank_command,
	.erase = fm3_erase,
	.write = fm3_write_block,
	.probe = fm3_probe,
	.auto_probe = fm3_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
