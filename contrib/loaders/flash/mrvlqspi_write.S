/***************************************************************************
 *   Copyright (C) 2014 by Mahavir Jain <mjain@marvell.com>                *
 *                                                                         *
 *   Adapted from (contrib/loaders/flash/lpcspifi_write.S):                *
 *   Copyright (C) 2012 by George Harris                                   *
 *   george@luminairecoffee.com                                            *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m3
	.thumb
	.thumb_func

/*
 * For compilation:
 * arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -c contrib/loaders/flash/mrvlqspi_write.S
 * arm-none-eabi-objcopy -O binary mrvlqspi_write.o code.bin
 * Copy code.bin into mrvlqspi flash driver
 */

/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address (offset from flash base)
 * r3 = count (bytes)
 * r4 = page size
 * r5 = qspi base address
 * Clobbered:
 * r7 - rp
 * r8 - wp, tmp
 * r9 - send/receive data
 * r10 - current page end address
 */

#define CNTL	0x0
#define CONF	0x4
#define DOUT	0x8
#define DIN	0xc
#define INSTR   0x10
#define ADDR    0x14
#define RDMODE  0x18
#define HDRCNT	0x1c
#define DINCNT  0x20

#define SS_EN (1 << 0)
#define XFER_RDY (1 << 1)
#define RFIFO_EMPTY (1 << 4)
#define WFIFO_EMPTY (1 << 6)
#define WFIFO_FULL (1 << 7)
#define FIFO_FLUSH (1 << 9)
#define RW_EN (1 << 13)
#define XFER_STOP (1 << 14)
#define XFER_START (1 << 15)

#define INS_WRITE_ENABLE 0x06
#define INS_READ_STATUS 0x05
#define INS_PAGE_PROGRAM 0x02

init:
	mov.w 	r10, #0x00
find_next_page_boundary:
	add 	r10, r4		/* Increment to the next page */
	cmp 	r10, r2
	/* If we have not reached the next page boundary after the target address, keep going */
	bls 	find_next_page_boundary
write_enable:
	/* Flush read/write fifos */
	bl 	flush_fifo

	/* Instruction byte 1 */
	movs 	r8, #0x1
	str 	r8, [r5, #HDRCNT]

	/* Set write enable instruction */
	movs 	r8, #INS_WRITE_ENABLE
	str 	r8, [r5, #INSTR]

	movs	r9, #0x1
	bl	start_tx
	bl 	stop_tx
page_program:
	/* Instruction byte 1, Addr byte 3 */
	movs 	r8, #0x31
	str 	r8, [r5, #HDRCNT]
	/* Todo: set addr and data pin to single */
write_address:
	mov 	r8, r2
	str 	r8, [r5, #ADDR]
	/* Set page program instruction */
	movs 	r8, #INS_PAGE_PROGRAM
	str 	r8, [r5, #INSTR]
	/* Start write transfer */
	movs	r9, #0x1
	bl	start_tx
wait_fifo:
	ldr 	r8, [r0]  	/* read the write pointer */
	cmp 	r8, #0 		/* if it's zero, we're gonzo */
	beq 	exit
	ldr 	r7, [r0, #4] 	/* read the read pointer */
	cmp 	r7, r8 		/* wait until they are not equal */
	beq 	wait_fifo
write:
	ldrb 	r9, [r7], #0x01 /* Load one byte from the FIFO, increment the read pointer by 1 */
	bl 	write_data 	/* send the byte to the flash chip */

	cmp 	r7, r1		/* wrap the read pointer if it is at the end */
	it  	cs
	addcs	r7, r0, #8	/* skip loader args */
	str 	r7, [r0, #4]	/* store the new read pointer */
	subs	r3, r3, #1	/* decrement count */
	cmp	r3, #0 		/* Exit if we have written everything */
	beq	write_wait
	add 	r2, #1 		/* Increment flash address by 1 */
	cmp 	r10, r2   	/* See if we have reached the end of a page */
	bne 	wait_fifo 	/* If not, keep writing bytes */
write_wait:
	bl 	stop_tx		/* Otherwise, end the command and keep going w/ the next page */
	add 	r10, r4 	/* Move up the end-of-page address by the page size*/
check_flash_busy:		/* Wait for the flash to finish the previous page write */
	/* Flush read/write fifos */
	bl 	flush_fifo
	/* Instruction byte 1 */
	movs 	r8, #0x1
	str 	r8, [r5, #HDRCNT]
	/* Continuous data in of status register */
	movs	r8, #0x0
	str 	r8, [r5, #DINCNT]
	/* Set write enable instruction */
	movs 	r8, #INS_READ_STATUS
	str 	r8, [r5, #INSTR]
	/* Start read transfer */
	movs	r9, #0x0
	bl	start_tx
wait_flash_busy:
	bl 	read_data
	and.w	r9, r9, #0x1
	cmp	r9, #0x0
	bne.n	wait_flash_busy
	bl 	stop_tx
	cmp	r3, #0
	bne.n 	write_enable 	/* If it is done, start a new page write */
	b	exit		/* All data written, exit */

write_data: 			/* Send/receive 1 byte of data over QSPI */
	ldr	r8, [r5, #CNTL]
	lsls    r8, r8, #24
	bmi.n	write_data
	str 	r9, [r5, #DOUT]
	bx	lr

read_data:			/* Read 1 byte of data over QSPI */
	ldr	r8, [r5, #CNTL]
	lsls    r8, r8, #27
	bmi.n	read_data
	ldr	r9, [r5, #DIN]
	bx	lr

flush_fifo:			/* Flush read write fifos */
	ldr	r8, [r5, #CONF]
	orr.w   r8, r8, #FIFO_FLUSH
	str     r8, [r5, #CONF]
flush_reset:
	ldr	r8, [r5, #CONF]
	lsls    r8, r8, #22
	bmi.n	flush_reset
	bx	lr

start_tx:
	ldr	r8, [r5, #CNTL]
	orr.w	r8, r8, #SS_EN
	str	r8, [r5, #CNTL]
xfer_rdy:
	ldr	r8, [r5, #CNTL]
	lsls    r8, r8, #30
	bpl.n	xfer_rdy
	ldr	r8, [r5, #CONF]
	bfi	r8, r9, #13, #1
	orr.w	r8, r8, #XFER_START
	str	r8, [r5, #CONF]
	bx lr

stop_tx:
	ldr	r8, [r5, #CNTL]
	lsls    r8, r8, #30
	bpl.n	stop_tx
wfifo_wait:
	ldr	r8, [r5, #CNTL]
	lsls    r8, r8, #25
	bpl.n	wfifo_wait
	ldr	r8, [r5, #CONF]
	orr.w	r8, r8, #XFER_STOP
	str	r8, [r5, #CONF]
xfer_start:
	ldr	r8, [r5, #CONF]
	lsls	r8, r8, #16
	bmi.n 	xfer_start
ss_disable:
	# Disable SS_EN
	ldr	r8, [r5, #CNTL]
	bic.w	r8, r8, #SS_EN
	str	r8, [r5, #CNTL]
wait:
	ldr	r8, [r5, #CNTL]
	lsls    r8, r8, #30
	bpl.n	wait
	bx 	lr

error:
	movs	r0, #0
	str 	r0, [r2, #4]	/* set rp = 0 on error */
exit:
	mov 	r0, r6
	bkpt 	#0x00

	.end
