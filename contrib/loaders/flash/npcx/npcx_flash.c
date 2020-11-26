/***************************************************************************
 *   Copyright (C) 2020 by Nuvoton Technology Corporation                  *
 *   Mulin Chao <mlchao@nuvoton.com>                                       *
 *   Wealian Liao <WHLIAO@nuvoton.com>                                     *
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

#include <stdint.h>
#include <string.h>
#include "npcx_flash.h"

/*----------------------------------------------------------------------------
 *                             NPCX flash driver
 *----------------------------------------------------------------------------*/
static void flash_execute_cmd(uint8_t code, uint8_t cts)
{
	/* Set UMA code */
	UMA_CODE = code;
	/* Execute UMA flash transaction by CTS setting */
	UMA_CTS  = cts;
	/* Wait for transaction completed */
	while (IS_BIT_SET(UMA_CTS, UMA_CTS_EXEC_DONE))
		;
}

static void flash_cs_level(uint8_t level)
{
	/* Program chip select pin to high/low level */
	if (level)
		SET_BIT(UMA_ECTS, UMA_ECTS_SW_CS1);
	else
		CLEAR_BIT(UMA_ECTS, UMA_ECTS_SW_CS1);
}

static void flash_set_address(uint32_t dest_addr)
{
	uint8_t *addr = (uint8_t *)&dest_addr;

	/* Set target flash address */
	UMA_AB2 = addr[2];
	UMA_AB1 = addr[1];
	UMA_AB0 = addr[0];
}

void delay(uint32_t i)
{
	while (i--)
		;
}

static int flash_wait_ready(uint32_t timeout)
{
	if (timeout <= 0)
		return STATUS_FAILED;

	/* Chip Select down. -- Burst mode */
	flash_cs_level(0);

	/* Command for Read status register */
	flash_execute_cmd(CMD_READ_STATUS_REG, MASK_CMD_ONLY);
	while (timeout > 0) {
		/* Read status register */
		UMA_CTS = MASK_RD_1BYTE;
		while (IS_BIT_SET(UMA_CTS, UMA_CTS_EXEC_DONE))
			;

		if (!(UMA_DB0 & SPI_FLASH_SR1_BUSY))
			break;

		if (--timeout > 0)
			delay(100);

	}; /* Wait for Busy clear */

	/* Chip Select high. */
	flash_cs_level(1);

	if (timeout == 0)
		return STATUS_FAILED_TIMEOUT;

	return STATUS_OK;
}

static int flash_write_enable(void)
{
	/* Write enable command */
	flash_execute_cmd(CMD_WRITE_EN, MASK_CMD_ONLY);

	/* Wait for flash is not busy */
	int status = flash_wait_ready(FLASH_ABORT_TIMEOUT);
	if (status != STATUS_OK)
		return status;

	if (UMA_DB0 & SPI_FLASH_SR1_WEL)
		return STATUS_OK;
	else
		return STATUS_FAILED;
}

static void flash_burst_write(uint32_t dest_addr, uint16_t bytes,
		const uint8_t *data)
{
	/* Chip Select down -- Burst mode */
	flash_cs_level(0);

	/* Set write address */
	flash_set_address(dest_addr);
	/* Start programming */
	flash_execute_cmd(CMD_FLASH_PROGRAM, MASK_CMD_WR_ADR);
	for (uint32_t i = 0; i < bytes; i++) {
		flash_execute_cmd(*data, MASK_CMD_WR_ONLY);
		data++;
	}

	/* Chip Select up */
	flash_cs_level(1);
}

/* The writing date couldn't cross 256Bytes boundary. */
static int flash_program_write(uint32_t addr, uint32_t size,
		const uint8_t *data)
{
	int status = flash_write_enable();
	if (status != STATUS_OK)
		return status;

	flash_burst_write(addr, size, data);
	status = flash_wait_ready(FLASH_ABORT_TIMEOUT);
	return status;
}

int flash_physical_write(uint32_t offset, uint32_t size, const uint8_t *data)
{
	int status;
	uint32_t trunk_start = (offset + 0xff) & ~0xff;

	/* write head */
	uint32_t dest_addr = offset;
	uint32_t write_len = ((trunk_start - offset) > size) ? size : (trunk_start - offset);

	if (write_len) {
		status = flash_program_write(dest_addr, write_len, data);
		if (status != STATUS_OK)
			return status;
		data += write_len;
	}

	dest_addr = trunk_start;
	size -= write_len;

	/* write remaining data*/
	while (size > 0) {
		write_len = (size > CONFIG_FLASH_WRITE_IDEAL_SIZE) ?
					CONFIG_FLASH_WRITE_IDEAL_SIZE : size;

		status = flash_program_write(dest_addr, write_len, data);
		if (status != STATUS_OK)
			return status;

		data      += write_len;
		dest_addr += write_len;
		size      -= write_len;
	}

	return STATUS_OK;
}

int flash_physical_erase(uint32_t offset, uint32_t size)
{
	/* Alignment has been checked in upper layer */
	for (; size > 0; size -= NPCX_MONITOR_FLASH_ERASE_SIZE,
		offset += NPCX_MONITOR_FLASH_ERASE_SIZE) {
		/* Enable write */
		int status = flash_write_enable();
		if (status != STATUS_OK)
			return status;

		/* Set erase address */
		flash_set_address(offset);
		/* Start erase */
		flash_execute_cmd(CMD_SECTOR_ERASE, MASK_CMD_ADR);
		/* Wait erase completed */
		status = flash_wait_ready(FLASH_ABORT_TIMEOUT);
		if (status != STATUS_OK)
			return status;
	}

	return STATUS_OK;
}

int flash_physical_erase_all(void)
{
	/* Enable write */
	int status = flash_write_enable();
	if (status != STATUS_OK)
		return status;

	/* Start erase */
	flash_execute_cmd(CMD_CHIP_ERASE, MASK_CMD_ONLY);

	/* Wait erase completed */
	status = flash_wait_ready(FLASH_ABORT_TIMEOUT);
	if (status != STATUS_OK)
		return status;

	return STATUS_OK;
}

int flash_physical_clear_stsreg(void)
{
	/* Enable write */
	int status = flash_write_enable();
	if (status != STATUS_OK)
		return status;

	UMA_DB0 = 0x0;
	UMA_DB1 = 0x0;

	/* Write status register 1/2 */
	flash_execute_cmd(CMD_WRITE_STATUS_REG, MASK_CMD_WR_2BYTE);

	/* Wait writing completed */
	status = flash_wait_ready(FLASH_ABORT_TIMEOUT);
	if (status != STATUS_OK)
		return status;

	/* Read status register 1/2 for checking */
	flash_execute_cmd(CMD_READ_STATUS_REG, MASK_CMD_RD_1BYTE);
	if (UMA_DB0 != 0x00)
		return STATUS_FAILED;
	flash_execute_cmd(CMD_READ_STATUS_REG2, MASK_CMD_RD_1BYTE);
	if (UMA_DB0 != 0x00)
		return STATUS_FAILED;

	return STATUS_OK;
}

int flash_get_id(uint32_t *id)
{
	flash_execute_cmd(CMD_READ_ID, MASK_CMD_RD_3BYTE);
	*id = UMA_DB0 << 16 | UMA_DB1 << 8 | UMA_DB2;

	return STATUS_OK;
}

/*----------------------------------------------------------------------------
 *                             flash loader function
 *----------------------------------------------------------------------------*/
uint32_t flashloader_init(struct flash_params *params)
{
	/* Initialize params buffers */
	memset((void *)params, 0, sizeof(struct flash_params));

	return STATUS_OK;
}

/*----------------------------------------------------------------------------
 *                                      Functions
 *----------------------------------------------------------------------------*/
#define BUFFER_LEN 4096 /* 4KB */

/* flashloader parameter structure */
__attribute__ ((section(".buffers.g_cfg")))
volatile struct flash_params g_cfg;
/* data buffer */
__attribute__ ((section(".buffers.g_buf")))
uint8_t g_buf[BUFFER_LEN];

int main(void)
{
	uint32_t id;

	/* set buffer */
	flashloader_init((struct flash_params *)&g_cfg);

	/* Avoid F_CS0 toggles while programming the internal flash. */
	SET_BIT(NPCX_DEVALT(0), NPCX_DEVALT0_NO_F_SPI);

	/* clear flash status registers */
	int status = flash_physical_clear_stsreg();
	if (status != STATUS_OK) {
		while (1)
			g_cfg.full = status;
	}

	while (1) {
		/* wait command*/
		while (g_cfg.full == LOADER_WAIT)
			;

		/* command handler */
		switch (g_cfg.cmd) {
			case CMD_GET_FLASH_ID:
				status = flash_get_id(&id);
				if (status == STATUS_OK) {
					g_buf[0] = id & 0xff;
					g_buf[1] = (id >> 8) & 0xff;
					g_buf[2] = (id >> 16) & 0xff;
					g_buf[3] = 0x00;
				}
				break;
			case CMD_ERASE_SECTORS:
				status = flash_physical_erase(g_cfg.dest, g_cfg.len);
				break;
			case CMD_ERASE_ALL:
				status = flash_physical_erase_all();
				break;
			case CMD_PROGRAM:
				status = flash_physical_write(g_cfg.dest,
								g_cfg.len,
								g_buf);
				break;
			default:
				status = STATUS_FAILED_UNKNOWN_COMMAND;
				break;
		}

		/* clear & set result for next command */
		if (status != STATUS_OK) {
			g_cfg.full = status;
			while (1)
				;
		} else {
			g_cfg.full = LOADER_WAIT;
		}
	}

	return 0;
}

__attribute__ ((section(".stack")))
__attribute__ ((used))
static uint32_t stack[100];
extern uint32_t _estack;
extern uint32_t _bss;
extern uint32_t _ebss;

__attribute__ ((section(".entry")))
void entry(void)
{
	/* set sp from end of stack */
	__asm(" ldr sp, =_estack - 4");

	main();

	__asm(" bkpt #0x00");
}
