/******************************************************************************
*
* Copyright (C) 2012-2018 Texas Instruments Incorporated - http://www.ti.com/
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

MEMORY {
	MAIN_FLASH (RX) : ORIGIN = 0x00000000, LENGTH = 0x00200000
	INFO_FLASH (RX) : ORIGIN = 0x00200000, LENGTH = 0x00008000
	SRAM_CODE_0(RWX): ORIGIN = 0x01000000, LENGTH = 0x00000110
	SRAM_CODE_1(RWX): ORIGIN = 0x01000110, LENGTH = 0x00000030
	SRAM_CODE_2(RWX): ORIGIN = 0x01000150, LENGTH = 0x00000040
	SRAM_CODE_3(RWX): ORIGIN = 0x01000190, LENGTH = 0x00001E70
	SRAM_CODE_4(RWX): ORIGIN = 0x01002000, LENGTH = 0x00000200
	SRAM_DATA  (RW) : ORIGIN = 0x20002000, LENGTH = 0x00001000
}

REGION_ALIAS("REGION_INTVECT", SRAM_CODE_0);
REGION_ALIAS("REGION_RESET", SRAM_CODE_1);
REGION_ALIAS("REGION_DESCRIPTOR", SRAM_CODE_2);
REGION_ALIAS("REGION_TEXT", SRAM_CODE_3);
REGION_ALIAS("REGION_BSS", SRAM_CODE_3);
REGION_ALIAS("REGION_DATA", SRAM_DATA);
REGION_ALIAS("REGION_STACK", SRAM_CODE_4);
REGION_ALIAS("REGION_HEAP", SRAM_DATA);
REGION_ALIAS("REGION_ARM_EXIDX", SRAM_CODE_3);
REGION_ALIAS("REGION_ARM_EXTAB", SRAM_CODE_3);

SECTIONS {
	/* section for the interrupt vector area */
	.intvecs : {
		KEEP (*(.intvecs))
	} > REGION_INTVECT

	PROVIDE (_vtable_base_address =
		DEFINED(_vtable_base_address) ? _vtable_base_address : 0x20000000);

	.vtable (_vtable_base_address) : AT (_vtable_base_address) {
		KEEP (*(.vtable))
	} > REGION_DATA

	.descriptor :{
		FILL(0x00000000);
		. = ORIGIN(REGION_DESCRIPTOR) + LENGTH(REGION_DESCRIPTOR) - 1;
		BYTE(0x00);
		__ROM_AT = .;
	} > REGION_DESCRIPTOR

	.reset : {
		KEEP(*(.reset))
	} > REGION_RESET AT> REGION_RESET

	.text : {
		CREATE_OBJECT_SYMBOLS
		KEEP (*(.text))
		*(.text.*)
		. = ALIGN(0x4);
		KEEP (*(.ctors))
		. = ALIGN(0x4);
		KEEP (*(.dtors))
		. = ALIGN(0x4);
		__init_array_start = .;
		KEEP (*(.init_array*))
		__init_array_end = .;
		KEEP (*(.init))
		KEEP (*(.fini*))
	} > REGION_TEXT AT> REGION_TEXT

	.rodata : {
		*(.rodata)
		*(.rodata.*)
	} > REGION_TEXT AT> REGION_TEXT

	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > REGION_ARM_EXIDX AT> REGION_ARM_EXIDX

	.ARM.extab : {
		KEEP (*(.ARM.extab* .gnu.linkonce.armextab.*))
	} > REGION_ARM_EXTAB AT> REGION_ARM_EXTAB

	__etext = .;

	.data : {
		__data_load__ = LOADADDR (.data);
		__data_start__ = .;
		KEEP (*(.data))
		KEEP (*(.data*))
		. = ALIGN (4);
		__data_end__ = .;
	} > REGION_DATA AT> REGION_TEXT

	.bss : {
		__bss_start__ = .;
		*(.shbss)
		KEEP (*(.bss))
		*(.bss.*)
		*(COMMON)
		. = ALIGN (4);
		__bss_end__ = .;
	} > REGION_BSS AT> REGION_BSS

	.heap : {
		__heap_start__ = .;
		end = __heap_start__;
		_end = end;
		__end = end;
		KEEP (*(.heap))
		__heap_end__ = .;
		__HeapLimit = __heap_end__;
	} > REGION_HEAP AT> REGION_HEAP

	.stack (NOLOAD) : ALIGN(0x8) {
		_stack = .;
		KEEP(*(.stack))
	} > REGION_STACK AT> REGION_STACK

	__stack_top = ORIGIN(REGION_STACK) + LENGTH(REGION_STACK);
	PROVIDE(__stack = __stack_top);
}
