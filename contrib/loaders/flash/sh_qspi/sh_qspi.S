/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * SH QSPI (Quad SPI) driver
 * Copyright (C) 2019 Marek Vasut <marek.vasut@gmail.com>
 */

#define BIT(n)		(1UL << (n))
/* SH QSPI register bit masks <REG>_<BIT> */
#define SPCR_MSTR	0x08
#define SPCR_SPE	0x40
#define SPSR_SPRFF	0x80
#define SPSR_SPTEF	0x20
#define SPPCR_IO3FV	0x04
#define SPPCR_IO2FV	0x02
#define SPPCR_IO1FV	0x01
#define SPBDCR_RXBC0	BIT(0)
#define SPCMD_SCKDEN	BIT(15)
#define SPCMD_SLNDEN	BIT(14)
#define SPCMD_SPNDEN	BIT(13)
#define SPCMD_SSLKP	BIT(7)
#define SPCMD_BRDV0	BIT(2)
#define SPCMD_INIT1	SPCMD_SCKDEN | SPCMD_SLNDEN | \
			SPCMD_SPNDEN | SPCMD_SSLKP | \
			SPCMD_BRDV0
#define SPCMD_INIT2	SPCMD_SPNDEN | SPCMD_SSLKP | \
			SPCMD_BRDV0
#define SPBFCR_TXRST	BIT(7)
#define SPBFCR_RXRST	BIT(6)
#define SPBFCR_TXTRG	0x30
#define SPBFCR_RXTRG	0x07

/* SH QSPI register set */
#define SH_QSPI_SPCR		0x00
#define SH_QSPI_SSLP		0x01
#define SH_QSPI_SPPCR		0x02
#define SH_QSPI_SPSR		0x03
#define SH_QSPI_SPDR		0x04
#define SH_QSPI_SPSCR		0x08
#define SH_QSPI_SPSSR		0x09
#define SH_QSPI_SPBR		0x0a
#define SH_QSPI_SPDCR		0x0b
#define SH_QSPI_SPCKD		0x0c
#define SH_QSPI_SSLND		0x0d
#define SH_QSPI_SPND		0x0e
#define SH_QSPI_DUMMY0		0x0f
#define SH_QSPI_SPCMD0		0x10
#define SH_QSPI_SPCMD1		0x12
#define SH_QSPI_SPCMD2		0x14
#define SH_QSPI_SPCMD3		0x16
#define SH_QSPI_SPBFCR		0x18
#define SH_QSPI_DUMMY1		0x19
#define SH_QSPI_SPBDCR		0x1a
#define SH_QSPI_SPBMUL0		0x1c
#define SH_QSPI_SPBMUL1		0x20
#define SH_QSPI_SPBMUL2		0x24
#define SH_QSPI_SPBMUL3		0x28

.syntax unified
.arm
.text

.macro wait_for_spsr, spsrbit
	1:	ldrb	r12, [r0, #SH_QSPI_SPSR]
		tst	r12, \spsrbit
		beq	1b
.endm

.macro sh_qspi_xfer
	bl	sh_qspi_cs_activate
	str	r6, [r0, SH_QSPI_SPBMUL0]
	bl	sh_qspi_xfer_common
	bl	sh_qspi_cs_deactivate
.endm

.macro sh_qspi_write_enable
	ldr	r4,	=SPIFLASH_WRITE_ENABLE
	adr	r5,	_start
	add	r4,	r5
	mov	r5,	#0x0
	mov	r6,	#0x1
	sh_qspi_xfer
.endm

.macro sh_qspi_wait_till_ready
	1:	ldr	r4,	=SPIFLASH_READ_STATUS
		adr	r5,	_start
		add	r4,	r5
		mov	r5,	#0x0
		mov	r6,	#0x2
		sh_qspi_xfer
		and	r13,	#0x1
		cmp	r13,	#0x1
		beq	1b
.endm

/*
 * r0: controller base address
 * r1: data buffer base address
 * r2: BIT(31) -- page program (not read)
 *     BIT(30) -- 4-byte address (not 3-byte)
 *     BIT(29) -- 512-byte page (not 256-byte)
 *     BIT(27:20) -- SF command
 *     BIT(19:0)  -- amount of data to read/write
 * r3: SF target address
 *
 * r7: data size
 * r8: page size
 *
 * r14: lr, link register
 * r15: pc, program counter
 *
 * Clobber: r4, r5, r6, r7, r8
 */

.global _start
_start:
	bic	r7,	r2, #0xff000000
	bic	r7,	r7, #0x00f00000

	and	r8,	r2, #(1 << 31)
	cmp	r8,	#(1 << 31)
	beq	do_page_program

/* fast read */

	bl	sh_qspi_cs_activate

	bl	sh_qspi_setup_command
	add	r8, r6, r7
	str	r8, [r0, SH_QSPI_SPBMUL0]
	bl	sh_qspi_xfer_common

	mov	r4,	#0x0
	mov	r5,	r1
	mov	r6,	r7
	bl	sh_qspi_xfer_common

	bl	sh_qspi_cs_deactivate

	b end

do_page_program:

	mov	r8,	#0x100
	tst	r2,	(1 << 29)
	movne	r8,	#0x200

do_pp_next_page:
	/* Check if less then page bytes left. */
	cmp	r7,	r8
	movlt	r8,	r7

	sh_qspi_write_enable

	bl	sh_qspi_cs_activate

	bl	sh_qspi_setup_command
	str	r6, [r0, SH_QSPI_SPBMUL0]
	bl	sh_qspi_xfer_common

	mov	r4,	r1
	mov	r5,	#0x0
	mov	r6,	r8

	bl	sh_qspi_xfer_common

	bl	sh_qspi_cs_deactivate

	sh_qspi_wait_till_ready

	add	r1,	r8
	add	r3,	r8
	sub	r7,	r8
	cmp	r7,	#0

	bne	do_pp_next_page

end:
	bkpt	#0

sh_qspi_cs_activate:
	/* Set master mode only */
	mov	r12,	#SPCR_MSTR
	strb	r12,	[r0, SH_QSPI_SPCR]

	/* Set command */
	mov	r12,	#SPCMD_INIT1
	strh	r12,	[r0, SH_QSPI_SPCMD0]

	/* Reset transfer and receive Buffer */
	ldrb	r12,	[r0, SH_QSPI_SPSCR]
	orr	r12,	#(SPBFCR_TXRST | SPBFCR_RXRST)
	strb	r12,	[r0, SH_QSPI_SPBFCR]

	/* Clear transfer and receive Buffer control bit */
	ldrb	r12,	[r0, SH_QSPI_SPBFCR]
	bic	r12,	#(SPBFCR_TXRST | SPBFCR_RXRST)
	strb	r12,	[r0, SH_QSPI_SPBFCR]

	/* Set sequence control method. Use sequence0 only */
	mov	r12,	#0x00
	strb	r12,	[r0, SH_QSPI_SPSCR]

	/* Enable SPI function */
	ldrb	r12,	[r0, SH_QSPI_SPCR]
	orr	r12,	#SPCR_SPE
	strb	r12,	[r0, SH_QSPI_SPCR]

	mov	pc,	lr

sh_qspi_cs_deactivate:
	/* Disable SPI function */
	ldrb	r12,	[r0, SH_QSPI_SPCR]
	bic	r12,	#SPCR_SPE
	strb	r12,	[r0, SH_QSPI_SPCR]

	mov	pc,	lr

/*
 * r0, controller base address
 * r4, tx buffer
 * r5, rx buffer
 * r6, xfer len, non-zero
 *
 * Upon exit, r13 contains the last byte in SPDR
 *
 * Clobber: r11, r12, r13
 */
sh_qspi_xfer_common:
prepcopy:
	ldr	r13, [r0, #SH_QSPI_SPBFCR]
	orr	r13, #(SPBFCR_TXTRG | SPBFCR_RXTRG)
	mov	r11, #32
	cmp	r6, #32

	biclt	r13, #(SPBFCR_TXTRG | SPBFCR_RXTRG)
	movlt	r11, #1

copy:
	str	r13, [r0, #SH_QSPI_SPBFCR]

	wait_for_spsr SPSR_SPTEF

	mov	r12, r11
	mov	r13, #0
	cmp	r4, #0
	beq	3f

2:	ldrb	r13, [r4], #1
	strb	r13, [r0, #SH_QSPI_SPDR]
	subs	r12, #1
	bne	2b
	b	4f

3:	strb	r13, [r0, #SH_QSPI_SPDR]
	subs	r12, #1
	bne	3b

4:	wait_for_spsr SPSR_SPRFF

	mov	r12, r11
	cmp	r5, #0
	beq	6f

5:	ldrb	r13, [r0, #SH_QSPI_SPDR]
	strb	r13, [r5], #1
	subs	r12, #1
	bne	5b
	b	7f

6:	ldrb	r13, [r0, #SH_QSPI_SPDR]
	subs	r12, #1
	bne	6b

7:	subs	r6, r11
	bne	prepcopy

	mov	pc,	lr

sh_qspi_setup_command:
	ldr	r4,	=SPIFLASH_SCRATCH_DATA
	adr	r5,	_start
	add	r4,	r5
	and	r12,	r2, #0x0ff00000
	lsr	r12,	#20
	strb	r12,	[r4]
	mov	r12,	r3
	strb	r12,	[r4, #4]
	lsr	r12,	#8
	strb	r12,	[r4, #3]
	lsr	r12,	#8
	strb	r12,	[r4, #2]
	lsr	r12,	#8
	strb	r12,	[r4, #1]
	lsr	r12,	#8
	mov	r5,	#0x0
	mov	r6,	#0x4
	tst	r2,	(1 << 30)
	movne	r6,	#0x5

	mov	pc,	lr

SPIFLASH_READ_STATUS:	.byte	0x05 /* Read Status Register */
SPIFLASH_WRITE_ENABLE:	.byte	0x06 /* Write Enable */
SPIFLASH_NOOP:		.byte	0x00
SPIFLASH_SCRATCH_DATA:	.byte	0x00, 0x0, 0x0, 0x0, 0x0
