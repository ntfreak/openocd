#define RA_WAIT			ra
#define RA_CLEAR_FIFO	sp
#define RA_EXEC_CMD		gp
#define RA_READ_STATUS	tp
#define RA_WRITE_ENABLE	s0
#define RA_WIP			s1

	.global _start
_start:
command_table:
	j	main	//address 0x0
	ebreak		//address 0x4
	j	erase	//address 0x8
	j	tx	//address 0xc

main:
	lbu		t0, 0(a0)
	addi	a0, a0, 1
	la		t1, command_table
	add		t0, t0, t1
	jr		t0

// 0xf0b00034:SPI Status Register
// Set t2 value to check status(ex SPIActive:set t2 0x1)
wait:
	li		a1, 0xf0b00034
	lw		a2, 0(a1)
	and		a2, a2, t2
	bnez	a2, wait
	jalr	x0, RA_WAIT, 0x0

// 0xf0b00030:SPI Control Register
// Reset RXFIFORST and TXFIFORST
clear_fifo:
	li		a1, 0xf0b00030
	lw		a2, 0(a1)
	ori		a2, a2, 0x2
	ori		a2, a2, 0x4
	sw		a2, 0(a1)
	jalr	x0, RA_CLEAR_FIFO, 0x0

// 0xf0b0002c:SPI Data Register
// 0xf0b00020:SPI Transfer Control Register
// 0xf0b00024:SPI Command Register
// Set t0 as flash command(ex write_enable command:set t0 0x6)
// Set t1 as transfer information(ex transfer mode:read/write, tx/rx count)
// Set dummy value to trigger SPI Transfer
exec_cmd:
	li		t2, 0x1
	jal		RA_WAIT, wait
	jal		RA_CLEAR_FIFO, clear_fifo
	li		a2, 0xf0b0002c
	sw		t0, 0(a2)
	li		a2, 0xf0b00020
	sw		t1, 0(a2)
	li		a1, 0x0
	li		a2, 0xf0b00024
	sw		a1, 0(a2)
	jalr	x0, RA_EXEC_CMD, 0x0

// Execute RDSR(read status register) command: set t0 to flash command 0x5,
// set t1 to transfer information 0x3000000
// (transfer mode:write, read and write count:0 + 1(command 0x5:1byte))
//
// After execute command, wait to read data : set t2 0x4000 to wait Receive FIFO not Empty
// Read status register value(from SPI Data Register) store at a2 and return
read_status:
	li		t0, 0x5
	li		t1, 0x3000000
	jal		RA_EXEC_CMD, exec_cmd
	li		t2, 0x4000
	jal		RA_WAIT, wait
	li		a1, 0xf0b0002c
	lw		a2, 0(a1)
	jalr	x0, RA_READ_STATUS, 0x0

// Execute WREN(write enable) command:
//	set t0 to flash command 0x6
//	set t1 to transfer information 0x1000000
//	(transfer mode:write only and write count:0 + 1(command 0x6:1byte))
//
// After execute WREN command, call read status function to get status register value(store at a2)
// Check bit 1(write enable latch) is 1 (means write enable)
write_enable:
	li		t0, 0x6
	li		t1, 0x1000000
	jal		RA_EXEC_CMD, exec_cmd
	jal		RA_READ_STATUS, read_status
	andi	a2, a2, 0x2
	beqz	a2, write_enable
	jalr	x0, RA_WRITE_ENABLE, 0x0

// Call read status function to get status register value(store at a2)
// Check status register value bit 0(write in progress bit) is 0 (means not in write operation)
wip:
	jal		RA_READ_STATUS, read_status
	andi	a2, a2, 0x1
	bnez	a2, wip
	jalr	x0, RA_WIP, 0x0

// Erase step:
//	set write enable and check ->
//	execute sector erase command ->
//	check not in write operation
//
// Execute SE(sector erase) command:
//	set t0 to flash command(read from address:a0)
//	set t1 to transfer information 0x1003000
//	(transfer mode:write only and write count:3 + 1(command t0:4bytes))
erase:
	jal		RA_WRITE_ENABLE, write_enable
	lw		t0, 0(a0)
	addi	a0, a0, 4
	li		t1, 0x1003000
	jal		RA_EXEC_CMD, exec_cmd
	jal		RA_WIP, wip
	j		main

// TX step:
//	set write enable and check ->
//	execute page program command ->
//	loop tx_word ->
//	check not in write operation
//
// Execute PP(page program) command:
//	set t0 to flash command(read from address:a0)
//	set t1 to transfer information 0x1103000
//	(transfer mode:write only and write count:0x103 + 1
//	(command t0:4bytes and tx data:0x100bytes(one page size)))
//
// tx_word:
//	set t2 0x800000 to wait Transmit FIFO not Full and
//	send one word tx data to SPI Data Register
tx:
	jal		RA_WRITE_ENABLE, write_enable
	lw		t0, 0(a0)
	addi	a0, a0, 4
	li		t1, 0x1103000
	jal		RA_EXEC_CMD, exec_cmd
	li		a3, 0x0
	li		a4, 0x40
tx_word:
	li		t2, 0x800000
	jal		RA_WAIT, wait
	lw		a1, 0(a0)
	addi	a0, a0, 4
	li		a2, 0xf0b0002c
	sw		a1, 0(a2)
	addi	a3, a3, 1
	bltu	a3, a4, tx_word
	jal		RA_WIP, wip
	j		main
