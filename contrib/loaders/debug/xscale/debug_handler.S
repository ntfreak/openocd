/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#include "protocol.h"

    .text
    .align  4

@ Disable thumb mode
    .code 32

@ send word to debugger
.macro m_send_to_debugger reg
1:
	mrc p14, 0, r15, c14, c0, 0
	bvs 1b
	mcr p14, 0, \reg, c8, c0, 0
.endm

@ receive word from debugger
.macro m_receive_from_debugger reg
1:
	mrc p14, 0, r15, c14, c0, 0
	bpl 1b
	mrc p14, 0, \reg, c9, c0, 0
.endm

@ save register on debugger, small
.macro m_small_save_reg reg
	mov r0, \reg
	bl send_to_debugger
.endm

@ save status register on debugger, small
.macro m_small_save_psr
	mrs r0, spsr
	bl send_to_debugger
.endm

@ wait for all outstanding coprocessor accesses to complete
.macro m_cpwait
	mrc p15, 0, r0, c2, c0, 0
	mov r0, r0
	sub pc, pc, #4
.endm

.global reset_handler
.global undef_handler
.global swi_handler
.global prefetch_abort_handler
.global data_abort_handler
.global irq_handler
.global fiq_handler

.section .part1 , "ax"

reset_handler:
	@ read DCSR
	mrc p14, 0, r13, c10, c0
	@ check if global enable bit (GE) is set
	ands r13, r13, #0x80000000

	bne debug_handler

	@ set global enable bit (GE)
	mov r13, #0xc0000000
	mcr p14, 0, r13, c10, c0

debug_handler:

	@ save r0 without modifying other registers
	m_send_to_debugger r0

	@ save lr (program PC) without branching (use macro)
	m_send_to_debugger r14

	@ save non-banked registers and spsr (program CPSR)
	m_small_save_reg r1
	m_small_save_reg r2
	m_small_save_reg r3
	m_small_save_reg r4
	m_small_save_reg r5
	m_small_save_reg r6
	m_small_save_reg r7
	m_small_save_psr

	mrs r0, spsr

	@ prepare program PSR for debug use (clear Thumb, set I/F to disable interrupts)
	bic r0, r0, #PSR_T
	orr r0, r0, #(PSR_I | PSR_F)

	@ examine mode bits
	and r1, r0, #MODE_MASK
	cmp r1, #MODE_USR

	bne not_user_mode

	@ replace USR mode with SYS
	bic r0, r0, #MODE_MASK
	orr r0, r0, #MODE_SYS

not_user_mode:

	b save_banked_registers

@ command loop
@ wait for command from debugger, than execute desired function
get_command:
	bl receive_from_debugger

	@ 0x0n - register access
	cmp r0, #0x0
	beq get_banked_registers

	cmp r0, #0x1
	beq set_banked_registers

	@ 0x1n - read memory
	cmp r0, #0x11
	beq read_byte

	cmp r0, #0x12
	beq read_half_word

	cmp r0, #0x14
	beq read_word

	@ 0x2n - write memory
	cmp r0, #0x21
	beq write_byte

	cmp r0, #0x22
	beq write_half_word

	cmp r0, #0x24
	beq write_word

	@ 0x3n - program execution
	cmp r0, #0x30
	beq resume

	cmp r0, #0x31
	beq resume_w_trace

	@ 0x4n - coprocessor access
	cmp r0, #0x40
	beq read_cp_reg

	cmp r0, #0x41
	beq write_cp_reg

	@ 0x5n - cache and mmu functions
	cmp r0, #0x50
	beq clean_d_cache

	cmp r0, #0x51
	beq invalidate_d_cache

	cmp r0, #0x52
	beq invalidate_i_cache

	cmp r0, #0x53
	beq cpwait

	@ 0x6n - misc functions
	cmp r0, #0x60
	beq clear_sa

	cmp r0, #0x61
	beq read_trace_buffer

	cmp r0, #0x62
	beq clean_trace_buffer

	@ return (back to get_command)
	b get_command

@ ----

@ resume program execution
resume:
	@ restore CPSR (SPSR_dbg)
	bl receive_from_debugger
	msr spsr, r0

	@ restore registers (r7 - r0)
	bl receive_from_debugger @ r7
	mov r7, r0
	bl receive_from_debugger @ r6
	mov r6, r0
	bl receive_from_debugger @ r5
	mov r5, r0
	bl receive_from_debugger @ r4
	mov r4, r0
	bl receive_from_debugger @ r3
	mov r3, r0
	bl receive_from_debugger @ r2
	mov r2, r0
	bl receive_from_debugger @ r1
	mov r1, r0
	bl receive_from_debugger @ r0

	@ resume addresss
	m_receive_from_debugger lr

	@ branch back to application code, restoring CPSR
	subs pc, lr, #0

@ get banked registers
@ receive mode bits from host, then run into save_banked_registers to

get_banked_registers:
	bl receive_from_debugger

@ save banked registers
@ r0[4:0]: desired mode bits
save_banked_registers:
	@ backup CPSR
	mrs r7, cpsr
	msr cpsr_c, r0
	nop

	@ keep current mode bits in r1 for later use
	and r1, r0, #MODE_MASK

	@ backup banked registers
	m_send_to_debugger r8
	m_send_to_debugger r9
	m_send_to_debugger r10
	m_send_to_debugger r11
	m_send_to_debugger r12
	m_send_to_debugger r13
	m_send_to_debugger r14

	@ if not in SYS mode (or USR, which we replaced with SYS before)
	cmp r1, #MODE_SYS

	beq no_spsr_to_save

	@ backup SPSR
	mrs r0, spsr
	m_send_to_debugger r0

no_spsr_to_save:

	@ restore CPSR for SDS
	msr cpsr_c, r7
	nop

   	@ return
	b get_command

@ ----


@ set banked registers
@ receive mode bits from host, then run into save_banked_registers to

set_banked_registers:
	bl receive_from_debugger

@ restore banked registers
@ r0[4:0]: desired mode bits
restore_banked_registers:
	@ backup CPSR
	mrs r7, cpsr
	msr cpsr_c, r0
	nop

	@ keep current mode bits in r1 for later use
	and r1, r0, #MODE_MASK

	@ set banked registers
	m_receive_from_debugger r8
	m_receive_from_debugger r9
	m_receive_from_debugger r10
	m_receive_from_debugger r11
	m_receive_from_debugger r12
	m_receive_from_debugger r13
	m_receive_from_debugger r14

	@ if not in SYS mode (or USR, which we replaced with SYS before)
	cmp r1, #MODE_SYS

	beq no_spsr_to_restore

	@ set SPSR
	m_receive_from_debugger r0
	msr spsr, r0

no_spsr_to_restore:

	@ restore CPSR for SDS
	msr cpsr_c, r7
	nop

   	@ return
	b get_command

@ ----

read_byte:
	@ r2: address
	bl receive_from_debugger
	mov r2, r0

	@ r1: count
	bl receive_from_debugger
	mov r1, r0

rb_loop:
	ldrb r0, [r2], #1

	@ drain write- (and fill-) buffer to work around XScale errata
	mcr p15, 0, r8, c7, c10, 4

	bl send_to_debugger

	subs r1, r1, #1
	bne rb_loop

	@ return
	b get_command

@ ----

read_half_word:
	@ r2: address
	bl receive_from_debugger
	mov r2, r0

	@ r1: count
	bl receive_from_debugger
	mov r1, r0

rh_loop:
	ldrh r0, [r2], #2

	@ drain write- (and fill-) buffer to work around XScale errata
	mcr p15, 0, r8, c7, c10, 4

	bl send_to_debugger

	subs r1, r1, #1
	bne rh_loop

	@ return
	b get_command

@ ----

read_word:
	@ r2: address
	bl receive_from_debugger
	mov r2, r0

	@ r1: count
	bl receive_from_debugger
	mov r1, r0

rw_loop:
	ldr r0, [r2], #4

	@ drain write- (and fill-) buffer to work around XScale errata
	mcr p15, 0, r8, c7, c10, 4

	bl send_to_debugger

	subs r1, r1, #1
	bne rw_loop

	@ return
	b get_command

@ ----

write_byte:
	@ r2: address
	bl receive_from_debugger
	mov r2, r0

	@ r1: count
	bl receive_from_debugger
	mov r1, r0

wb_loop:
	bl receive_from_debugger
	strb r0, [r2], #1

	@ drain write- (and fill-) buffer to work around XScale errata
	mcr p15, 0, r8, c7, c10, 4

	subs r1, r1, #1
	bne wb_loop

	@ return
	b get_command

@ ----

write_half_word:
	@ r2: address
	bl receive_from_debugger
	mov r2, r0

	@ r1: count
	bl receive_from_debugger
	mov r1, r0

wh_loop:
	bl receive_from_debugger
	strh r0, [r2], #2

	@ drain write- (and fill-) buffer to work around XScale errata
	mcr p15, 0, r8, c7, c10, 4

	subs r1, r1, #1
	bne wh_loop

	@ return
	b get_command

@ ----

write_word:
	@ r2: address
	bl receive_from_debugger
	mov r2, r0

	@ r1: count
	bl receive_from_debugger
	mov r1, r0

ww_loop:
	bl receive_from_debugger
	str r0, [r2], #4

	@ drain write- (and fill-) buffer to work around XScale errata
	mcr p15, 0, r8, c7, c10, 4

	subs r1, r1, #1
	bne ww_loop

	@ return
	b get_command

@ ----

clear_sa:
	@ read DCSR
	mrc p14, 0, r0, c10, c0

	@ clear SA bit
	bic r0, r0, #0x20

	@ write DCSR
	mcr p14, 0, r0, c10, c0

	@ return
	b get_command

@ ----

clean_d_cache:
	@ r0: cache clean area
	bl receive_from_debugger

	mov r1, #1024
clean_loop:
	mcr p15, 0, r0, c7, c2, 5
	add r0, r0, #32
	subs r1, r1, #1
	bne clean_loop

	@ return
	b get_command

@ ----

invalidate_d_cache:
	mcr p15, 0, r0, c7, c6, 0

	@ return
	b get_command

@ ----

invalidate_i_cache:
	mcr p15, 0, r0, c7, c5, 0

	@ return
	b get_command

@ ----

cpwait:
	m_cpwait

	@return
	b get_command

@ ----

.section .part2 , "ax"

read_cp_reg:
	@ requested cp register
	bl receive_from_debugger

	adr r1, read_cp_table
	add pc, r1, r0, lsl #3

read_cp_table:
	mrc p15, 0, r0, c0, c0, 0  @ XSCALE_MAINID
	b read_cp_reg_reply
	mrc p15, 0, r0, c0, c0, 1  @ XSCALE_CACHETYPE
	b read_cp_reg_reply
	mrc p15, 0, r0, c1, c0, 0  @ XSCALE_CTRL
	b read_cp_reg_reply
	mrc p15, 0, r0, c1, c0, 1  @ XSCALE_AUXCTRL
	b read_cp_reg_reply
	mrc p15, 0, r0, c2, c0, 0  @ XSCALE_TTB
	b read_cp_reg_reply
	mrc p15, 0, r0, c3, c0, 0  @ XSCALE_DAC
	b read_cp_reg_reply
	mrc p15, 0, r0, c5, c0, 0  @ XSCALE_FSR
	b read_cp_reg_reply
	mrc p15, 0, r0, c6, c0, 0  @ XSCALE_FAR
	b read_cp_reg_reply
	mrc p15, 0, r0, c13, c0, 0  @ XSCALE_PID
	b read_cp_reg_reply
	mrc p15, 0, r0, c15, c0, 0  @ XSCALE_CP_ACCESS
	b read_cp_reg_reply
	mrc p15, 0, r0, c14, c8, 0  @ XSCALE_IBCR0
	b read_cp_reg_reply
	mrc p15, 0, r0, c14, c9, 0  @ XSCALE_IBCR1
	b read_cp_reg_reply
	mrc p15, 0, r0, c14, c0, 0  @ XSCALE_DBR0
	b read_cp_reg_reply
	mrc p15, 0, r0, c14, c3, 0  @ XSCALE_DBR1
	b read_cp_reg_reply
	mrc p15, 0, r0, c14, c4, 0  @ XSCALE_DBCON
	b read_cp_reg_reply
	mrc p14, 0, r0, c11, c0, 0 @ XSCALE_TBREG
	b read_cp_reg_reply
	mrc p14, 0, r0, c12, c0, 0 @ XSCALE_CHKPT0
	b read_cp_reg_reply
	mrc p14, 0, r0, c13, c0, 0 @ XSCALE_CHKPT1
	b read_cp_reg_reply
	mrc p14, 0, r0, c10, c0, 0 @ XSCALE_DCSR
	b read_cp_reg_reply

read_cp_reg_reply:
	bl send_to_debugger

	@ return
	b get_command

@ ----

write_cp_reg:
	@ requested cp register
	bl receive_from_debugger
	mov r1, r0

	@ value to be written
	bl receive_from_debugger

	adr r2, write_cp_table
	add pc, r2, r1, lsl #3

write_cp_table:
	mcr p15, 0, r0, c0, c0, 0  @ XSCALE_MAINID (0x0)
	b get_command
	mcr p15, 0, r0, c0, c0, 1  @ XSCALE_CACHETYPE (0x1)
	b get_command
	mcr p15, 0, r0, c1, c0, 0  @ XSCALE_CTRL (0x2)
	b get_command
	mcr p15, 0, r0, c1, c0, 1  @ XSCALE_AUXCTRL (0x3)
	b get_command
	mcr p15, 0, r0, c2, c0, 0  @ XSCALE_TTB (0x4)
	b get_command
	mcr p15, 0, r0, c3, c0, 0  @ XSCALE_DAC (0x5)
	b get_command
	mcr p15, 0, r0, c5, c0, 0  @ XSCALE_FSR (0x6)
	b get_command
	mcr p15, 0, r0, c6, c0, 0  @ XSCALE_FAR (0x7)
	b get_command
	mcr p15, 0, r0, c13, c0, 0  @ XSCALE_PID (0x8)
	b get_command
	mcr p15, 0, r0, c15, c0, 0  @ XSCALE_CP_ACCESS (0x9)
	b get_command
	mcr p15, 0, r0, c14, c8, 0  @ XSCALE_IBCR0 (0xa)
	b get_command
	mcr p15, 0, r0, c14, c9, 0  @ XSCALE_IBCR1 (0xb)
	b get_command
	mcr p15, 0, r0, c14, c0, 0  @ XSCALE_DBR0 (0xc)
	b get_command
	mcr p15, 0, r0, c14, c3, 0  @ XSCALE_DBR1 (0xd)
	b get_command
	mcr p15, 0, r0, c14, c4, 0  @ XSCALE_DBCON (0xe)
	b get_command
	mcr p14, 0, r0, c11, c0, 0 @ XSCALE_TBREG (0xf)
	b get_command
	mcr p14, 0, r0, c12, c0, 0 @ XSCALE_CHKPT0 (0x10)
	b get_command
	mcr p14, 0, r0, c13, c0, 0 @ XSCALE_CHKPT1 (0x11)
	b get_command
	mcr p14, 0, r0, c10, c0, 0 @ XSCALE_DCSR (0x12)
	b get_command

@ ----

read_trace_buffer:

	@ dump 256 entries from trace buffer
	mov	r1, #256
read_tb_loop:
	mrc p14, 0, r0, c11, c0, 0 @ XSCALE_TBREG
	bl send_to_debugger
	subs r1, r1, #1
	bne read_tb_loop

	@ dump checkpoint register 0
	mrc p14, 0, r0, c12, c0, 0 @ XSCALE_CHKPT0 (0x10)
	bl send_to_debugger

	@ dump checkpoint register 1
	mrc p14, 0, r0, c13, c0, 0 @ XSCALE_CHKPT1 (0x11)
	bl send_to_debugger

	@ return
	b get_command

@ ----

clean_trace_buffer:

	@ clean 256 entries from trace buffer
	mov	r1, #256
clean_tb_loop:
	mrc p14, 0, r0, c11, c0, 0 @ XSCALE_TBREG
	subs r1, r1, #1
	bne clean_tb_loop

	@ return
	b get_command

@ ----


@ resume program execution with trace buffer enabled
resume_w_trace:
	@ restore CPSR (SPSR_dbg)
	bl receive_from_debugger
	msr spsr, r0

	@ restore registers (r7 - r0)
	bl receive_from_debugger @ r7
	mov r7, r0
	bl receive_from_debugger @ r6
	mov r6, r0
	bl receive_from_debugger @ r5
	mov r5, r0
	bl receive_from_debugger @ r4
	mov r4, r0
	bl receive_from_debugger @ r3
	mov r3, r0
	bl receive_from_debugger @ r2
	mov r2, r0
	bl receive_from_debugger @ r1
	mov r1, r0
	bl receive_from_debugger @ r0

	@ resume addresss
	m_receive_from_debugger lr

	mrc p14, 0, r13, c10, c0, 0 @ XSCALE_DCSR
	orr r13, r13, #1
	mcr p14, 0, r13, c10, c0, 0 @ XSCALE_DCSR

	@ branch back to application code, restoring CPSR
	subs pc, lr, #0

undef_handler:
swi_handler:
prefetch_abort_handler:
data_abort_handler:
irq_handler:
fiq_handler:
1:
	b 1b

send_to_debugger:
	m_send_to_debugger r0
	mov pc, lr

receive_from_debugger:
	m_receive_from_debugger r0
	mov pc, lr
