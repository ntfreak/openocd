#include "../../../../src/flash/nor/spi.h"
#include "../../../../src/flash/nor/cmspi.h"

/* set NCS */
	.macro deselect
deselect:
	ldr		r6, port_pin_ncs			/* NCS port address */
	ldr		r7, [r6]					/* load port data */
	ldr		r6, port_pin_ncs+4			/* NCS pin bitmask */
	orrs	r7, r7, r6					/* set NCS bit */
	ldr		r6, port_pin_ncs			/* NCS port address */
	str		r7, [r6]					/* store new contents */
	bx		lr							/* return */
	.endm

/* prepare common regs */
	.macro	setup_regs
	subs	r0, #1						/* decrement count */
	mov		r11, r0						/* save count */
	mov		r12, r2						/* save address offset */
	subs	r1, #1						/* create page size mask */
	lsls	r3, r3, #29					/* addr size into bits 29-31 */
	orrs	r1, r1, r3					/* copy size in page size mask */
	mov		r10, r1						/* save mask and flags */
	.endm

/* entry point, initialize registers for SPI mode */
	.macro	setup_regs_spi
	setup_regs
	ldr		r0, port_pin_io1			/* load IO1/MISO port address */
	ldr		r1, cmd						/* input offset (bits 16-23) */
	lsrs	r1, r1, #((inp_off-cmd)<<3)	/* shift into lowest byte */
	sxtb	r1, r1						/* sign-extend 8-bit offset */
	lsls	r1, r1, #2					/* word offset to byte offset */
	adds	r1, r1, r0					/* calculate input port address */
	adr		r0, port_pin_io3			/* fetch address, io3 unused here */
	str		r1, [r0]					/* save input port address */
	ldr		r0, port_pin_clk			/* load CLK port address */
	ldr		r1, port_pin_clk+4			/* load CLK pin bitmask */
	.endm

/* send cmd and following 1-, 2-, 3- or 4-byte address to flash  */
	.macro	send_cmd_addr	type
	bl		shift_out_byte_\type		/* send cmd */
	mov		r7, r10						/* get address size */
	lsrs	r7, r7, #29					/* address size in bytes */
	cmp		r7, #1						/* 1-byte address ? */
	bls		addr_0_7					/* then skip bits 8-31 */
	cmp		r7, #2						/* 2-byte address ? */
	bls		addr_0_15					/* then skip bits 16-31 */
	cmp		r7, #3						/* 3-byte address ? */
	bls		addr_0_23					/* then skip bits 23-31 */
addr_0_31:
	mov		r4, r12						/* get address offset */
	lsrs	r4, r4, #24					/* addr bits 31-24 */
	bl		shift_out_next_\type		/* send addr byte */
addr_0_23:
	mov		r4, r12						/* get address offset */
	lsrs	r4, r4, #16					/* addr bits 23-16 */
	bl		shift_out_next_\type		/* send addr byte */
addr_0_15:
	mov		r4, r12						/* get address offset */
	lsrs	r4, r4, #8					/* addr bits 15-8 */
	bl		shift_out_next_\type		/* send addr byte */
addr_0_7:
	mov		r4, r12						/* get address offset */
	lsrs	r4, r4, #0					/* addr bits 7-0 */
	bl		shift_out_next_\type		/* send addr byte */
	.endm

/* toggle direction of IO0 to IO3 pin as far as requested
 * clobbered: r3, r4, r5, r6, r7 */
	.macro	toggle_dir
toggle_dir:
	cmp		r3, r4						/* test for read/write or set/clr cap. */
	adr		r4, port_pin_io0			/* pointer to IO0 port address */
	bne		setclr_dir_loop
toggle_dir_loop:
	ldmia	r4!, {r5, r6}				/* load port addr and exor mask */
	sxtb	r7, r3						/* extend 8-bit to 32-bit offset */
	lsls	r7, r7, #2					/* word offset to byte offset, test offset */
	beq		toggle_dir_next				/* if offset equals 0, then nothing to do */
	adds	r5, r5, r7					/* pointer to dir. register */
	ldr		r7, [r5]					/* get direction register */
	eors	r7, r7, r6					/* invert direction bit(s) */
	str		r7, [r5]					/* store new direction reg. */
toggle_dir_next:
	lsrs	r3, r3, #8					/* prepare for next pin */
	bne		toggle_dir_loop				/* again for next pin, if any */
	bx		lr							/* return */
setclr_dir_loop:
	ldmia	r4!, {r5, r6}				/* load port addr and set/clr mask */
	sxtb	r7, r3						/* extend 8-bit to 32-bit offset */
	lsls	r7, r7, #2					/* word offset to byte offset, test offset */
	beq		setclr_dir_next				/* if offset equals 0, then nothing to do */
	adds	r5, r5, r7					/* pointer to dir. register */
	str		r6, [r5]					/* store new direction reg. */
setclr_dir_next:
	lsrs	r3, r3, #8					/* prepare for next pin */
	bne		setclr_dir_loop				/* again for next pin, if any */
	bx		lr							/* return */
	.endm

	.macro	set_to_input
	ldr		r3, dir_inp_off				/* to input first */
	ldr		r4, dir_out_off				/* to output last */
	bl		toggle_dir					/* switch IO1-IO0 to input */
	.endm

	.macro	set_to_output
	ldr		r3, dir_out_off				/* to output first */
	ldr		r4, dir_inp_off				/* to input last */
	bl		toggle_dir					/* switch IO1-IO0 to output */
	.endm

/* prepare shift count */
	.macro	prepare_shift_in_spi
prepare_shift_in_spi:
	ldr		r2, port_pin_io3			/* IO1/MISO *input* port address */
	ldr		r5, bit_no					/* bit numbers, IO1/MISO in bits 8-15 */
	lsls	r5, r5, #(31-15)			/* clear bits 16-31 */
	lsrs	r5, r5, #(31-7)				/* shift IO1/MISO bit no. in bits 0-7 */
	adds	r5, r5, #1					/* to shift into C */
	.endm

/* shift in one byte in SPI mode
 * r0: port CLK output addr
 * r1: CLK bit mask
 * r2: port IO1/MISO input addr
 * r4: data byte (out)
 * r5: MISO shift count (in/out)
 * clobbered: r4, r6, r7 */
	.macro	shift_in_byte_spi
shift_in_byte_spi:
	movs	r4, #(1<<0)					/* set bit 0 only */
	lsls	r4, r4, #24					/* into bit 24 */
	ldr		r6, [r0]					/* load CLK port data */
shift_in_loop_spi:
	dsb									/* adjust duty cycle */
	orrs	r6, r6, r1					/* set CLK bit */
	str		r6, [r0]					/* store new CLK port data */
	bics	r6, r6, r1					/* clear CLK bit */
	ldr		r7, [r2]					/* load IO1/MISO input data */
	str		r6, [r0]					/* store new CLK port data */
	lsrs	r7, r7, r5					/* IO1/MISO bit into bit C */
	adcs	r4, r4						/* shift left and insert new */
	bcc		shift_in_loop_spi			/* again if not finished */
	.endm

/* tail of shift_out_byte, IO0/MOSI port address parametrized */
	.macro	shift_tail	mosi
	lsls	r4, r4, #1					/* original bit 7 into C */
shift_out_loop_\@:
	bics	r6, r6, r1					/* clear CLK bit */
	sbcs	r5, r5, r5					/* fill all bits with ~C */
	ands	r5, r5, r3					/* set/clear IO0/MOSI bit */
	str		r6, [r0]					/* store new CLK port data */
	orrs	\mosi, \mosi, r3			/* set IO0/MOSI bit */
	bics	\mosi, \mosi, r5			/* insert new IO0/MOSI bit */
	str		\mosi, [r2]					/* store IO0/MOSI port data */
	orrs	r6, r6, r1					/* set CLK bit */
	lsls	r4, r4, #1					/* shift next bit into C */
	str		r6, [r0]					/* store new CLK port data */
	bne		shift_out_loop_\@			/* again if not finished */
	str		r6, [r0]					/* store new CLK port data */
	bics	r6, r6, r1					/* clear CLK bit */
	str		r6, [r0]					/* store new CLK port data */
	bx		lr							/* return */
	.endm

/* clear NCS, shift out lowest byte of word in SPI mode
 * depending on whether IO0/MOSI and CLK are located on same port,
 * algorithm differs slightly
 *
 * r4: data byte (in)
 * clobbered: r2, r3, r5, r6, r7 */
	.macro	shift_out_byte_spi
shift_out_byte_spi:
	ldr		r2, port_pin_ncs			/* NCS port address */
	ldr		r3, port_pin_ncs+4			/* NCS pin bitmask */
	ldr		r7, [r2]					/* load port data */
	bics	r7, r7, r3					/* clear NCS bit */
	str		r7, [r2]					/* store new contents */
	ldr		r2, bit_no					/* bit no., IO0/MOSI in bits 0-7 */
	lsls	r2, r2, #(31-7)				/* clear bits 8-31 */
	lsrs	r2, r2, #(31-7)				/* shift IO0/MOSI in bits 0-7 */
	movs	r3, #1						/* create bit mask for */
	lsls	r3, r3, r2					/* IO0/MOSI pin */
	ldr		r2, port_pin_io0			/* IO0/MOSI port address */
shift_out_next_spi:
	lsls	r4, r4, #1					/* space for '1', bit 7 into bit 8 */
	adds	r4, r4, #1					/* insert '1' right of data */
	lsls	r4, r4, #(31-8)				/* original bit 7 into bit 31 */
	ldr		r6, [r0]					/* load CLK port data */
	bics	r6, r6, r1					/* clear CLK bit */
	cmp		r0, r2						/* CLK and IO0/MOSI on same port? */
	beq		shift_out_eq_spi
	ldr		r7, [r2]					/* load IO0/MOSI port data */
	shift_tail	mosi=r7					/* CLK, IO0/MOSI on diff. ports */
shift_out_eq_spi:
	shift_tail	mosi=r6					/* CLK, IO0/MOSI on same port */
	.endm

/* entry point, initialize registers for QPI mode */
	.macro	setup_regs_qpi
	setup_regs
	ldr		r0, port_pin_clk			/* load CLK port address */
	ldr		r1, port_pin_clk+4			/* load CLK pin bitmask */
	.endm

/* entry point, initialize registers for DPI mode */
	.macro	setup_regs_dpi
	setup_regs_qpi
	adr		r4, bit_no					/* get address of bit_no */
	ldr		r5, [r4]					/* fetch bit_no */
	lsls	r6, r5, #16					/* duplicate lower into upper half */
	uxth	r5, r5						/* clear upper half */
	orrs	r5, r5, r6					/* join duplicates */
	str		r5, [r4]					/* store for later use */
	.endm

/* get one bit from port and insert it in data byte */
	.macro	shift_in_bit
	rors	r5, r5, r2					/* bit number for next bit */
	ldr		r6, [r6, r3]				/* fetch port data */
	lsrs	r6, r6, r5					/* shift data bit into C */
	adcs	r4, r4, r4					/* insert C and shift */
	.endm

/* shift in one byte in DPI mode
 * r4: data byte (out)
 * clobbered: r2, r3, r5, r6, r7 */
	.macro	shift_in_byte_dpi
shift_in_byte_dpi:
	movs	r2, #(32-8)					/* rotate left count */
	ldr		r3, cmd						/* input offset (bits 16 to 23) */
	lsrs	r3, r3, #((inp_off-cmd)<<3)	/* shift into lowest byte */
	sxtb	r3, r3						/* sign-extend 8-bit offset */
	lsls	r3, r3, #2					/* word offset to byte offset */
	movs	r4, #1						/* set bit 0 */
	lsls	r4, r4, #23					/* shift into bit 23 */
	ldr		r5, bit_no					/* get bit numbers */
	ldr		r6, =0x01010101				/* get increment */
	adds	r5, r5, r6					/* increment all bit numbers */
	ldr		r7, [r0]					/* load CLK port data */
pair_in_loop:
	dsb									/* adjust duty cycle */
	orrs	r7, r7, r1					/* set CLK bit */
	str		r7, [r0]					/* store new CLK port data */
	ldr		r6, port_pin_io1			/* get port address for IO1 */
	shift_in_bit						/* read data bit */
	ldr		r6, port_pin_io0			/* get port address for IO0 */
	shift_in_bit						/* read data bit */
	bics	r7, r7, r1					/* clear CLK bit */
	tst		r4, r4						/* byte completed ? */
	str		r7, [r0]					/* store new CLK port data */
	bpl		pair_in_loop				/* if not, then next pair */
	bx		lr							/* return */
	.pool
	.endm

/* insert one bit from data byte into port */
	.macro	shift_out_bit
	sbcs	r3, r3, r3					/* set/clear all depending on C */
	ldr		r7, [r6]					/* fetch port data */
	movs	r5, #1						/* set bit 0 */
	lsls	r5, r5, r2					/* bit mask for port */
	orrs	r7, r7, r5					/* set port bit */
	ands	r5, r5, r3					/* create clear mask */
	bics	r7, r7, r5					/* clear port bit conditionally */
	str		r7, [r6]					/* store new port data */
	lsrs	r2, r2, #8					/* next bit number */
	.endm

/* clear NCS, shift out lowest byte of word in DPI mode
 *
 * r4: data byte (in)
 * clobbered: r2, r3, r5, r6, r7 */
	.macro	shift_out_byte_dpi
shift_out_byte_dpi:
	ldr		r2, port_pin_ncs			/* NCS port address */
	ldr		r3, port_pin_ncs+4			/* NCS pin bitmask */
	ldr		r7, [r2]					/* load port data */
	bics	r7, r7, r3					/* clear NCS bit */
	str		r7, [r2]					/* store new contents */
shift_out_next_dpi:
	lsls	r4, r4, #1					/* space for '1', bit 7 into bit 8 */
	adds	r4, r4, #1					/* insert '1' right of data */
	lsls	r4, r4, #(32-8)				/* shift data bit 7 into bit 31 */
pair_out_loop:
	ldr		r2, bit_no					/* get bit numbers */
	rev16	r2, r2						/* reverse numbers for IO1 and IO0 */
	ldr		r6, port_pin_io1			/* get port address for IO1 */
	shift_out_bit						/* store data bit */
	lsls	r4, r4, #1					/* next data bit into C */
	ldr		r6, port_pin_io0			/* get port address for IO0 */
	shift_out_bit						/* store data bit */
	ldr		r7, [r0]					/* load CLK port data */
	orrs	r7, r7, r1					/* set CLK bit */
	str		r7, [r0]					/* store new CLK port data */
	bics	r7, r7, r1					/* clear CLK bit */
	str		r7, [r0]					/* store new CLK port data */
	lsls	r4, r4, #1					/* next data bit into C */
	bne		pair_out_loop				/* if not finished, then next */
	bx		lr							/* return */
	.endm

/* shift in one byte in QPI mode
 * r4: data byte (out)
 * clobbered: r2, r3, r5, r6, r7 */
	.macro	shift_in_byte_qpi
shift_in_byte_qpi:
	movs	r2, #(32-8)					/* rotate left count */
	ldr		r3, cmd						/* input offset (bits 16 to 23) */
	lsrs	r3, r3, #((inp_off-cmd)<<3)	/* shift into lowest byte */
	sxtb	r3, r3						/* sign-extend 8-bit offset */
	lsls	r3, r3, #2					/* word offset to byte offset */
	movs	r4, #1						/* set bit 0 */
	lsls	r4, r4, #23					/* shift into bit 23 */
	ldr		r5, bit_no					/* get bit numbers */
	ldr		r6, =0x01010101				/* get increment */
	adds	r5, r5, r6					/* increment all bit numbers */
	ldr		r7, [r0]					/* load CLK port data */
nibble_in_loop:
	dsb									/* adjust duty cycle */
	orrs	r7, r7, r1					/* set CLK bit */
	str		r7, [r0]					/* store new CLK port data */
	ldr		r6, port_pin_io3			/* get port address for IO3 */
	shift_in_bit						/* read data bit */
	ldr		r6, port_pin_io2			/* get port address for IO2 */
	shift_in_bit						/* read data bit */
	ldr		r6, port_pin_io1			/* get port address for IO1 */
	shift_in_bit						/* read data bit */
	ldr		r6, port_pin_io0			/* get port address for IO0 */
	shift_in_bit						/* read data bit */
	bics	r7, r7, r1					/* clear CLK bit */
	tst		r4, r4						/* byte completed ? */
	str		r7, [r0]					/* store new CLK port data */
	bpl		nibble_in_loop				/* if not, then next nibble */
	bx		lr							/* return */
	.pool
	.endm

/* clear NCS, shift out lowest byte of word in QPI mode
 *
 * r4: data byte (in)
 * clobbered: r2, r3, r5, r6, r7 */
	.macro	shift_out_byte_qpi
shift_out_byte_qpi:
	ldr		r2, port_pin_ncs			/* NCS port address */
	ldr		r3, port_pin_ncs+4			/* NCS pin bitmask */
	ldr		r7, [r2]					/* load port data */
	bics	r7, r7, r3					/* clear NCS bit */
	str		r7, [r2]					/* store new contents */
shift_out_next_qpi:
	lsls	r4, r4, #1					/* space for '1', bit 7 into bit 8 */
	adds	r4, r4, #1					/* insert '1' right of data */
	lsls	r4, r4, #(32-8)				/* shift data bit 7 into bit 31 */
nibble_out_loop:
	ldr		r2, bit_no					/* get bit numbers */
	rev		r2, r2						/* reverse bit numbers */
	ldr		r6, port_pin_io3			/* get port address for IO3 */
	shift_out_bit						/* store data bit */
	lsls	r4, r4, #1					/* next data bit into C */
	ldr		r6, port_pin_io2			/* get port address for IO2 */
	shift_out_bit						/* store data bit */
	lsls	r4, r4, #1					/* next data bit into C */
	ldr		r6, port_pin_io1			/* get port address for IO1 */
	shift_out_bit						/* store data bit */
	lsls	r4, r4, #1					/* next data bit into C */
	ldr		r6, port_pin_io0			/* get port address for IO0 */
	shift_out_bit						/* store data bit */
	ldr		r7, [r0]					/* load CLK port data */
	orrs	r7, r7, r1					/* set CLK bit */
	str		r7, [r0]					/* store new CLK port data */
	bics	r7, r7, r1					/* clear CLK bit */
	str		r7, [r0]					/* store new CLK port data */
	lsls	r4, r4, #1					/* next data bit into C */
	bne		nibble_out_loop				/* if not finished, then next */
	bx		lr							/* return */
	.endm

	.macro dummy_clocks
	ldr		r4, cmd						/* load dummy (in bits 8 to 15) */
	lsrs	r4, r4, #((dummy-cmd)<<3)	/* shift into place */
	uxtb	r4, r4						/* clear upper 24 bits */
	tst		r4, r4						/* no. of dummy cyles non-zero ? */
dummy_loop:
	beq		dummy_end					/* if count at zero, then done */
	ldr		r3, [r0]					/* load CLK port data (delay) */
	ldr		r3, [r0]					/* load CLK port data */
	orrs	r3, r3, r1					/* set CLK bit */
	str		r3, [r0]					/* store new CLK port data */
	ldr		r3, [r0]					/* load CLK port data (delay) */
	ldr		r3, [r0]					/* load CLK port data */
	bics	r3, r3, r1					/* clear CLK bit */
	str		r3, [r0]					/* store new CLK port data */
	subs	r4, #1						/* decrement loop counter */
	b		dummy_loop					/* test if another iteration */
dummy_end:
	.endm

/* entry point, initialize registers for I2C mode */
	.macro	setup_regs_i2c
	setup_regs
	ldr		r0, cmd						/* input offset (bits 16 to 23) */
	lsrs	r0, r0, #((inp_off-cmd)<<3)	/* shift into lowest byte */
	sxtb	r0, r0						/* sign-extend 8-bit offset */
	lsls	r0, r0, #2					/* word offset to byte offset */
	ldr		r1, dir_inp_off				/* get dir_inp offsets */
	.endm

/* internal macro */
	.macro	_i2c_set_clr	addr1 addr2 cond off_reg shift
	ldr		r2, dir_out_off				/* get dir_out offsets */
	ldr		r3, port_pin_\addr1			/* address of SDA/SCL port */
	ldr		r5,	[r3]					/* get port data */
	ldr		r6,	port_pin_\addr1+4		/* get SDA/SCL bit mask */
	tst		r5, r6						/* SDA/SCL already set/cleared ? */
	\cond	2f							/* nothing to do if yes */
	eors	r5, r5, r6					/* toggle SDA/SCL bit */
	str		r5, [r3]					/* store new port data */
	lsrs	r6, \off_reg, #\shift		/* dir offset for SDA/SCL */
	sxtb	r6, r6						/* sign extend offset */
	lsls	r6, r6, #2					/* word offset into byte offset */
	ldr		r5, port_pin_io\addr2+4		/* get exor or set/clr mask */
	cmp		r1, r2						/* read/write or set/clr? */
	bne		1f							/* skip fetch and eor if set/clr */
	ldr		r7, [r3, r6]				/* fetch port config */
	eors	r5, r5, r7					/* toggle direction */
1:
	str		r5, [r3, r6]				/* store port config */
2:
	ldr		r2, cmd						/* dummy (bits 8 to 15) */
	lsrs	r2, r2, #((dummy-cmd)<<3)	/* shift into lowest byte */
	uxtb	r2, r2						/* clear upper bytes */
	lsls	r2, r2, #3					/* multiply by 8 */
3:
	movs	r3, r3						/* nop */
	subs	r2, r2, #1					/* decrement loop counter */
	bpl		3b							/* delay passed ? */
	.endm

/* these following four macros clobber r2, r3, r5, r6, r7 */
	.macro	i2c_clr_sda
	_i2c_set_clr	SPI_SDA I2C_SDA beq r2 8
	.endm

	.macro	i2c_set_sda
	_i2c_set_clr	SPI_SDA I2C_SDA bne r1 8
	.endm

	.macro	i2c_clr_scl
	_i2c_set_clr	SPI_SCL I2C_SCL beq r2 0
	.endm

	.macro	i2c_set_scl
	_i2c_set_clr	SPI_SCL I2C_SCL bne r1 0
	.endm

/* insert up to three topmost address bits into I2C address byte */
	.macro	prepare_i2c_cmd
	mov		r5, r12						/* get address offset */
	mov		r6, r10						/* get addr size in bytes */
	lsrs	r6, r6, #(29-3)				/* *bits* in address bytes */
	lsrs	r5, r5, r6					/* remaining address bits */
	lsls	r5, r5, #1					/* shift bits into bit 1-3 */
	orrs	r4, r4, r5					/* insert into I2C address */
	.endm

/* send cmd and following 1-, 2-, 3- or 4-byte address to flash in I2C mode */
	.macro	send_cmd_addr_i2c target
	prepare_i2c_cmd
	bl		i2c_start_cond				/* generate start condition */
	bl		shift_out_byte_i2c			/* send I2C address byte */
	bcs		\target						/* error exit if NACK */
	mov		r7, r10						/* get address size */
	lsrs	r7, r7, #29					/* address size in bytes */
	cmp		r7, #1						/* 1-byte address ? */
	bls		addr_0_7					/* then skip bits 8-31 */
	cmp		r7, #2						/* 2-byte address ? */
	bls		addr_0_15					/* then skip bits 16-31 */
	cmp		r7, #3						/* 3-byte address ? */
	bls		addr_0_23					/* then skip bits 23-31 */
addr_0_31:
	mov		r4, r12						/* get address offset */
	lsrs	r4, r4, #24					/* addr bits 31-24 */
	bl		shift_out_byte_i2c			/* send addr byte */
	bcs		error1						/* error exit if NACK */
addr_0_23:
	mov		r4, r12						/* get address offset */
	lsrs	r4, r4, #16					/* addr bits 23-16 */
	bl		shift_out_byte_i2c			/* send addr byte */
	bcs		error1						/* error exit if NACK */
addr_0_15:
	mov		r4, r12						/* get address offset */
	lsrs	r4, r4, #8					/* addr bits 15-8 */
	bl		shift_out_byte_i2c			/* send addr byte */
	bcs		error1						/* error exit if NACK */
addr_0_7:
	mov		r4, r12						/* get address offset */
	lsrs	r4, r4, #0					/* addr bits 7-0 */
	bl		shift_out_byte_i2c			/* send addr byte */
	bcs		error1						/* error exit if NACK */
	.endm

/* generate i2c start condition */
	.macro	i2c_start_cond
i2c_start_cond:
	i2c_set_scl
	i2c_clr_sda
	i2c_clr_scl
	bx		lr							/* return */
	.endm

/* generate i2c stop condition */
	.macro	i2c_stop_cond
i2c_stop_cond:
	i2c_clr_sda
	i2c_set_scl
	i2c_set_sda
	bx		lr							/* return */
	.endm

/* generate start condition, shift out lowest byte of word in I2C mode
 * and simultaneously shift in one byte
 * r4: data byte (in)
 * clobbered: r2, r3, r5, r6, r7 */
	.macro	shift_out_byte_i2c
shift_out_byte_i2c:
	lsls	r4, r4, #1					/* space for one bit, bit 7 into bit 8 */
	adds	r4, r4, #1					/* insert NACK bit to the right of data */
shift_in_start_i2c:
	lsls	r4, r4, #(31-8)				/* shift original data bit 7 into bit 31 */
	adds	r4, r4, #1					/* insert stop mark */
shift_out_loop_i2c:
	lsls	r4, r4, #1					/* new data bit into C */
	bcs		shift_set_sda				/* if data bit equals 1, then set SDA */
	i2c_clr_sda							/* else clear SDA */
	b		shift_out_bit_i2c			/* and continue */
shift_set_sda:
	i2c_set_sda							/* set SDA */
shift_out_bit_i2c:
	i2c_set_scl							/* set SCL */
	ldr		r2, port_pin_ncs			/* address of SDA port output reg */
	ldr		r3,	port_pin_ncs+4			/* get SDA bit mask */
	ldr		r2, [r2, r0]				/* fetch input register */
	tst		r2, r3						/* test SDA bit */
	beq		shift_in_bit_i2c			/* if cleared, skip */
	adds	r4, r4, #1					/* insert SDA bit */
shift_in_bit_i2c:
	i2c_clr_scl							/* clear SCL */
	lsrs	r5, r4, #10					/* stop mark in bit 9 already ? */
	bcc		shift_out_loop_i2c			/* again if byte not completed */
	lsrs	r4, r4, #1					/* ACK/NACK bit into C */
	bx		lr							/* return */
	.endm

/* parameter and buffer allocation */
	.macro	param_block
	.align	2							/* align to word */
cmd:
	.byte	0							/* READ or PPROG command, I2C address byte */
dummy:
	.byte	0							/* number of dummy clock cycles (only for READ) */
inp_off:
	.byte	0							/* offset port output reg. to port input reg. */
empty:
	.byte	0							/* keep word alignment */
bit_no:
	.word	0							/* bit numbers of IO3/NHOLD, IO2/NWP, IO1/MISO, IO0/MOSI */
dir_inp_off:
	.word	0							/* offsets port output reg. to port direction *CLR* reg. */
dir_out_off:
	.word	0							/* offsets port output reg. to port direction *SET* reg. */
port_pin_ncs:
	.word	0							/* port address (output reg) for NCS */
	.word	0							/* bit mask for NCS */
port_pin_clk:
	.word	0							/* port address (output reg) for CLK */
	.word	0							/* bit mask for CLK */
port_pin_io0:
	.word	0							/* port address (output reg) for IO0/MOSI/SCL */
	.word	0							/* exor mask for IO0/MOSI/SCL */
port_pin_io1:
	.word	0							/* port address (output reg) for IO1/MISO/SDA */
	.word	0							/* exor mask for IO1/MISO/SDA */
port_pin_io2:
	.word	0							/* port address (output reg) for IO2/NWP */
	.word	0							/* exor mask for IO2/NWP */
port_pin_io3:
	.word	0							/* port address (output reg) for IO3/NHOLD */
	.word	0							/* exor mask for IO3/NHOLD */
	.endm
