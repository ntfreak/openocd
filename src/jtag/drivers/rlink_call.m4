m4_divert(`-1')
/***************************************************************************
 *   Copyright (C) 2008 Lou Deluxe                                         *
 *   lou.openocd012@fixit.nospammail.net                                   *
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

m4_dnl Setup and hold times depend on SHIFTER_PRESCALER
m4_define(`SETUP_DELAY_CYCLES', m4_eval(`('SHIFTER_PRESCALER` + 1) / 2'))
m4_define(`HOLD_DELAY_CYCLES', m4_eval(`'SHIFTER_PRESCALER` / 2'))

m4_dnl Some macros to make nybble handling a little easier
m4_define(`m4_high_nybble', `m4_eval(`(($1) >> 4) & 0xf')')
m4_define(`m4_low_nybble', `m4_eval(`($1) & 0xf')')

m4_dnl A macro to generate a number of NOPs depending on the argument
m4_define(`m4_0_to_5_nops', `m4_ifelse(m4_eval(`($1) >= 1'), 1, `	NOP
'm4_ifelse(m4_eval(`($1) >= 2'), 1, `	NOP
'm4_ifelse(m4_eval(`($1) >= 3'), 1, `	NOP
'm4_ifelse(m4_eval(`($1) >= 4'), 1, `	NOP
'm4_ifelse(m4_eval(`($1) >= 5'), 1, `	NOP
')))))')


m4_dnl Some macros to facilitate bit-banging delays.
m4_dnl There are 3 of them.  One for self-contained delays, and two for those which must be split between setup and loop to keep from disturbing A at delay time.
m4_dnl The argument passed to any of them is the number of cycles which the delay should consume.

m4_dnl This one is self-contained.

m4_define(`m4_delay',
`; delay (m4_eval($1) cycles)'
`m4_ifelse(m4_eval(`('$1`) < 6'), 1,
	m4_0_to_5_nops($1)
,
	m4_ifelse(m4_eval(`(('$1`) - 3) % 2'), 1, `	NOP')
	A.H = m4_high_nybble(`(('$1`) - 3) / 2')
	A.L = m4_low_nybble(`(('$1`) - 3) / 2')
	Y = A
	DECY
	JP -1
)')


m4_dnl These are the setup and loop parts of the split delay.
m4_dnl The argument passed to both must match for the result to make sense.
m4_dnl The setup does not figure into the delay.  It takes 3 cycles when a loop is used and none if nops are used.

m4_define(`m4_delay_setup',
`; delay setup (m4_eval($1) cycles)'
`m4_ifelse(m4_eval(`('$1`) < 6'), 0, `	'
	A.H = m4_high_nybble(`('$1`) / 2')
	A.L = m4_low_nybble(`('$1`) / 2')
	Y = A
)')

m4_define(`m4_delay_loop',
`; delay loop (m4_eval($1) cycles)'
`m4_ifelse(m4_eval(`('$1`) < 6'), 1,
	m4_0_to_5_nops($1)
,
	m4_ifelse(m4_eval(`('$1`) % 2'), 1, `	NOP')
	DECY
	JP -1
)')

m4_dnl These are utility macros for use with delays.  Specifically, there is code below which needs some predictability in code size for relative jumps to reach.  The m4_delay macro generates an extra NOP when an even delay is needed, and the m4_delay_loop macro generates an extra NOP when an odd delay is needed.  Using this for the argument to the respective macro rounds up the argument so that the extra NOP will not be generated.  There is also logic built in to cancel the rounding when the result is small enough that a loop would not be generated.

m4_define(`m4_delay_loop_round_up', `m4_ifelse(m4_eval($1` < 6'), 1, $1, m4_eval(`(('$1`) + 1) / 2 * 2'))')
m4_define(`m4_delay_round_up', `m4_ifelse(m4_eval($1` < 6'), 1, $1, m4_eval(`(('$1`) / 2 * 2) + 1'))')


m4_divert(`0')m4_dnl

;------------------------------------------------------------------------------
:opcode_error
; This is at address 0x00 in case of empty LUT entries
	STATUS STOP ERROR

;------------------------------------------------------------------------------
; Command interpreter at address 0x01 because it is branched to a lot and having it be 0x01 means we can use X for it, which is already used for other purposes which want it to be 1.
; Assumes X is 1
; Assumes ADR_BUFFER0 points to the next command byte
; Stores the current command byte in CMP01

:command_interpreter
	A = DATA_BUFFER0
	ADR_BUFFER0 += X
	CMP01 = A	; store the current command for later

	EXCHANGE	; put MSN into LSN
	A.H = 0xc	; lookup table at 0x1550 + 0xc0 = 0x1610

	; branch to address in lookup table
	Y = A
	A = <Y>
	BRANCH

;------------------------------------------------------------------------------
; LUT for high nybble

;LUT; c0 opcode_error
;LUT; c1 opcode_shift_tdi_andor_tms_bytes
;LUT; c2 opcode_shift_tdi_andor_tms_bytes
;LUT; c3 opcode_shift_tdi_andor_tms_bytes
;LUT; c4 opcode_shift_tdo_bytes
;LUT; c5 opcode_error
;LUT; c6 opcode_shift_tdio_bytes
;LUT; c7 opcode_error
;LUT; c8 opcode_shift_tms_tdi_bit_pair
;LUT; c9 opcode_shift_tms_bits
;LUT; ca opcode_error
;LUT; cb opcode_error
;LUT; cc opcode_error
;LUT; cd opcode_error
;LUT; ce opcode_shift_tdio_bits
;LUT; cf opcode_stop


;------------------------------------------------------------------------------
; USB/buffer handling
;

;ENTRY; download entry_download

opcode_stop:
opcode_next_buffer:
	; pointer to completion flag
	A.H = 0xf
	A.L = 0xf
	Y = A

	A = OR_MPEG	; buffer indicator from previous iteration
	<Y> = A		; either indicator will have bit 0 set
	BSET 1		; was buffer 1 previously current?
;	A.H = 0		; already zero from OR_MPEG
	JP opcode_next_buffer_0

opcode_next_buffer_1:
	A.L = 0x1	; ack buffer 0
	BUFFER_MNGT = A
;	A.H = 0x0	; already zero from BUFFER_MNGT
	A.L = 0x3	; Input buffer 1 = 0x1850 (0x0300)
	JP +4

opcode_next_buffer_0:
	A.L = 0x2	; ack buffer 1
	BUFFER_MNGT = A
entry_download:
	A = X		; Input buffer 0 = 0x1650 (0x0100)

	ADR_BUFFER01 = A
	OR_MPEG = A	; store for next iteration

	A.L = 0x0
	BUFFER_MNGT = A	; finish acking previous buffer
	Y = A
	ADR_BUFFER00 = A
	ADR_BUFFER11 = A

	A.H = 0x4	; Output buffer = 0x1590 (0x0040)
	ADR_BUFFER10 = A

	EXCHANGE	; 0x04
	X = A		; for the spin loop below

	; pointer to status in shared memory
	DECY		; setting to 0 above and decrementing here saves a byte

	; wait until a command buffer is available
	A = BUFFER_MNGT	; spin while neither of bits 2 or 3 are set
	CP A<X		; this is slightly faster and smaller than trying to AND and compare the result, and it lets us just use the nybble-swapped 0x40 from the output buffer setup.
	JP -2
	<Y> = A		; update status once done spinning

	; restore X, since we used it
;	A.H = 0	; high nybble of BUFFER_MNGT will always be 0 the way we use it
	A.L = 1
	X = A

	; go to command interpreter
	BRANCH


;;------------------------------------------------------------------------------
;:opcode_stop
;;
;
;	; Ack buffer 0 in download mode
;	A.L = 0x1
;	BUFFER_MNGT = A
;
;	STATUS STOP


;------------------------------------------------------------------------------
:opcode_shift_tdi_andor_tms_bytes
;

	A = CMP01	; bits 3..0 contain the number of bytes to shift - 1
	A.H = 0
	Y = A		; loop counter

	A = CMP01
	EXCHANGE
	CMP01 = A	; we're interested in bits in the high nybble

opcode_shift_tdi_andor_tms_bytes__loop:

; set tdi to supplied byte or zero
	A = CMP01
	BSET 1
	JP +4
	A.H = 0
	A.L = 0
	JP +3
	A = DATA_BUFFER0
	ADR_BUFFER0 += X
	SHIFT_MPEG = A

; set tms to supplied byte or zero
	A = CMP01
	BCLR 0
	JP +5
	A = DATA_BUFFER0
	ADR_BUFFER0 += X
	SHIFT_CARD = A
	SHIFT CARD OUT=>PIN0

; run both shifters as nearly simultaneously as possible
	SHIFT MPEG OUT=>PIN1

	A = CTRL_FCI
	EXCHANGE
	BCLR 3
	JP -3

	DECY
	JP opcode_shift_tdi_andor_tms_bytes__loop

	A = X
	BRANCH


;------------------------------------------------------------------------------
:opcode_shift_tdo_bytes
;

	A = CMP01	; bits 3..0 contain the number of bytes to shift - 1
	A.H = 0
	Y = A		; loop counter

opcode_shift_tdo_bytes__loop:
	SHIFT MPEG PIN0=>IN

	A = CTRL_FCI
	EXCHANGE
	BCLR 3
	JP -3

	; put shifted byte into output buffer
	A = SHIFT_MPEG
	DATA_BUFFER1 = A
	ADR_BUFFER1 += X

	DECY
	JP opcode_shift_tdo_bytes__loop

	A = X
	BRANCH


;------------------------------------------------------------------------------
:opcode_shift_tdio_bytes
;

	A = CMP01	; bits 3..0 contain the number of bytes to shift - 1
	A.H = 0
	CMP10 = A	; byte loop counter

	A.H = opcode_shift_tdio_bytes__sub_return
	A.L = opcode_shift_tdio_bytes__sub_return
	CMP00 = A	; return address

opcode_shift_tdio_bytes__loop:
	A.H = 0
	A.L = 7
	CMP11 = A		; always use 8 bits

	JP sub_shift_tdio_bits
opcode_shift_tdio_bytes__sub_return:

	A = CMP10	; byte loop counter
	CP A=>X
	CLC
	A -= X
	CMP10 = A
	JP opcode_shift_tdio_bytes__loop

	A = X
;DR_MPEG = A ; return TCK low, as str912 reset halt seems to require it
	BRANCH


;------------------------------------------------------------------------------
:opcode_shift_tdio_bits
;

	A = CMP01	; bits 2..0 contain the number of bits to shift - 1
	A.H = 0
	BCLR 3		; set TMS=1 if bit 3 was set
	CMP11 = A	; bit loop counter

	A.H = opcode_shift_tdio_bits__sub_return
	A.L = opcode_shift_tdio_bits__sub_return
	CMP00 = A	; return address

	JP sub_shift_tdio_bits
	A.L = 0x1	; TMS=1
	DR_CARD = A
	JP sub_shift_tdio_bits
opcode_shift_tdio_bits__sub_return:

	A = X
;DR_MPEG = A ; return TCK low, as str912 reset halt seems to require it
	BRANCH


;------------------------------------------------------------------------------
:sub_shift_tdio_bits
;

	A = DATA_BUFFER0	; get byte from input buffer
	ADR_BUFFER0 += X
	MASK = A		; put it in MASK where bit routine will use it

:sub_shift_tdio_bits__loop
m4_delay_setup(m4_delay_loop_round_up(SETUP_DELAY_CYCLES - 1))

	A = MASK	; shift TDO into and TDI out of MASK via carry
	A += MASK
	MASK = A

	; shifting out TDI
	A.L = 0x2	; TCK=0, TDI=1
	CP CARRY
	JP +2
	A.L = 0x0	; TCK=0, TDI=0
	DR_MPEG = A

m4_delay_loop(m4_delay_loop_round_up(SETUP_DELAY_CYCLES - 1))

	BSET 2		; TCK high
	DR_MPEG = A

	A = DR_MPEG	; set carry bit to TDO
	CLC
	BCLR 0
	JP +2
	SEC

m4_delay(HOLD_DELAY_CYCLES - 10)

	A = CMP11	; bit loop counter
	Y = A		; use Y to avoid corrupting carry bit with subtract
	DECY
	A = Y
	CMP11 = A
	JP :sub_shift_tdio_bits__loop

	; shift last TDO bit into result
	A = MASK
	A += MASK
	DATA_BUFFER1 = A
	ADR_BUFFER1 += X

	A = CMP00	; return to caller
	BRANCH


;------------------------------------------------------------------------------
:opcode_shift_tms_tdi_bit_pair
;

; set TMS line manually
	A = CMP01	; bits 3..0 contain TDI and TMS bits and whether to return TDO
	BSET 0		; TMS bit
	A.L = 0x1	; TMS=1
	JP +2
	A.L = 0x0	; TMS=0
	DR_CARD = A

; stuff command buffer with bitmap of single TDI bit
	A = CMP01
	BSET 1		; TDI bit
	A.H = 0x8	; TDI=1
	JP +2
	A.H = 0x0	; TDI=0
	ADR_BUFFER0 -= X
	DATA_BUFFER0 = A

	A.H = 0
	A.L = 0
	CMP11 = A	; bit loop counter (only doing one bit)

	A.H = opcode_shift_tms_tdi_bit_pair__sub_return
	A.L = opcode_shift_tms_tdi_bit_pair__sub_return
	CMP00 = A	; return address

; jump this way due to relative jump range issues
	A.H = sub_shift_tdio_bits
	A.L = sub_shift_tdio_bits
	BRANCH
opcode_shift_tms_tdi_bit_pair__sub_return:

	A = CMP01
	BSET 3		; bit says whether to return TDO
	JP +2
	ADR_BUFFER1 -= X	; subroutine returns it, so undo that

	A = X
	DR_MPEG = A ; return TCK low, as str912 reset halt seems to require it
	BRANCH


;------------------------------------------------------------------------------
:opcode_shift_tms_bits
;

	A = CMP01	; bits 3..0 contain the number of bits to shift - 1 (only 1-8 bits is valid... no checking, just improper operation)
	A.H = 0
	CMP11 = A	; bit loop counter

	A = DATA_BUFFER0	; get byte from input buffer
	ADR_BUFFER0 += X
	MASK = A		; The byte we'll be shifting

:opcode_shift_tms_bits__loop
m4_delay_setup(SETUP_DELAY_CYCLES - 1)

	A = MASK	; shift TMS out of MASK via carry
	A += MASK
	MASK = A

	; shifting out TMS
	A.L = 0x1	; TCK=0, TDI=0, TMS=1
	CP CARRY
	JP +2
	A.L = 0x0	; TCK=0, TDI=0, TMS=0
	DR_CARD = A
	DR_MPEG = A

m4_delay_loop(SETUP_DELAY_CYCLES - 1)

	BSET 2		; TCK high
	DR_MPEG = A

m4_delay(HOLD_DELAY_CYCLES - 10)

	A = CMP11	; bit loop counter
	CP A=>X
	CLC
	A -= X
	CMP11 = A
	JP :opcode_shift_tms_bits__loop

	A = X
	DR_MPEG = A ; return TCK low, as str912 reset halt seems to require it
	BRANCH
