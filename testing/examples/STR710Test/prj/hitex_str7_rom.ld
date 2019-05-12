/***********************************************************************************
*	Copyright 2005 Anglia Design
*	This demo code and associated components are provided as is and has no warranty,
*	implied or otherwise.  You are free to use/modify any of the provided
*	code at your own risk in your applications with the expressed limitation
*	of liability (see below)
* 
*	LIMITATION OF LIABILITY:   ANGLIA OR ANGLIA DESIGNS SHALL NOT BE LIABLE FOR ANY
*	LOSS OF PROFITS, LOSS OF USE, LOSS OF DATA, INTERRUPTION OF BUSINESS, NOR FOR
*	INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES OF ANY KIND WHETHER UNDER
*	THIS AGREEMENT OR OTHERWISE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*
*	Author			: Spencer Oliver
*	Web     		: www.anglia-designs.com
*
***********************************************************************************/

/* Stack Sizes */

	_STACKSIZE     = 1024;
	_STACKSIZE_IRQ = 256;
	_STACKSIZE_FIQ = 0;
	_STACKSIZE_SVC = 1024;
	_STACKSIZE_ABT = 0;
	_STACKSIZE_UND = 0;
	_HEAPSIZE      = 1024;

/* Memory Definitions */

MEMORY
{
	CODE (rx) : ORIGIN = 0x40000000, LENGTH = 0x00040000
	DATA (rw) : ORIGIN = 0x20000000, LENGTH = 0x00010000
}

/* Section Definitions */

SECTIONS
{
	/* first section is .text which is used for code */

	.text :
	{
		CREATE_OBJECT_SYMBOLS
		KEEP(*(.vectrom))
		KEEP(*(.init))
		*(.text .text.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7t) *(.glue_7) *(.vfp11_veneer)
		KEEP(*(.fini))
		*(.gcc_except_table)
	} >CODE =0
	. = ALIGN(4);

	/* .ctors .dtors are used for c++ constructors/destructors */
	
	.ctors :
	{
		PROVIDE(__ctors_start__ = .);
		KEEP(*(SORT(.ctors.*)))
		KEEP(*(.ctors))
		PROVIDE(__ctors_end__ = .);
	} >CODE

	.dtors :
	{
		PROVIDE(__dtors_start__ = .); 
		KEEP(*(SORT(.dtors.*)))
		KEEP(*(.dtors))
		PROVIDE(__dtors_end__ = .);
	} >CODE
	
	/* .rodata section which is used for read-only data (constants) */

	.rodata :
	{
		*(.rodata .rodata.*)
		*(.gnu.linkonce.r.*)
	} >CODE
	. = ALIGN(4);

	.init_array :
	{
		*(.init)
        *(.fini)
		PROVIDE (__preinit_array_start = .);
		KEEP (*(.preinit_array))
		PROVIDE (__preinit_array_end = .);
		PROVIDE (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		PROVIDE (__init_array_end = .);
		PROVIDE (__fini_array_start = .);
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		PROVIDE (__fini_array_end = .);
	} >CODE

	. = ALIGN(4);
	
	/* .ARM.exidx is sorted, so has to go in its own output section.  */
	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} >CODE
	__exidx_end = .;

	_vectext = .;
	PROVIDE (vectext = .);

	.vect : AT (_vectext)
	{
		_vecstart = .;
		KEEP(*(.vectram))
		_vecend = .;
	} >DATA

	_etext = _vectext + SIZEOF(.vect);
	PROVIDE (etext = .);

	/* .data section which is used for initialized data */

	.data : AT (_etext)
	{
		__data_start = .;
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		SORT(CONSTRUCTORS)
		. = ALIGN(4);
		*(.fastrun .fastrun.*)
	} >DATA
	. = ALIGN(4);
	
	_edata = .;
	PROVIDE (edata = .);

	/* .bss section which is used for uninitialized data */

	.bss :
	{
		__bss_start = .;
		__bss_start__ = .;
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
	} >DATA
	. = ALIGN(4);
	__bss_end__ = .;
	
	_end = .;
	PROVIDE(end = .);

	/* .heap section which is used for memory allocation */
	
	.heap (NOLOAD) :
	{
		__heap_start__ = .;
		*(.heap)
		. = MAX(__heap_start__ + _HEAPSIZE , .);
	} >DATA
	__heap_end__ = __heap_start__ + SIZEOF(.heap);
	
	/* .stack section - user mode stack */
	
	.stack (__heap_end__ + 3) / 4 * 4 (NOLOAD) :
	{
		__stack_start__ = .;
		*(.stack)
		. = MAX(__stack_start__ + _STACKSIZE , .);
	} >DATA
	__stack_end__ = __stack_start__ + SIZEOF(.stack);

	/* .stack_irq section */
	
	.stack_irq (__stack_end__ + 3) / 4 * 4 (NOLOAD) :
	{
		__stack_irq_start__ = .;
		*(.stack_irq)
		. = MAX(__stack_irq_start__ + _STACKSIZE_IRQ , .);
	} >DATA
	__stack_irq_end__ = __stack_irq_start__ + SIZEOF(.stack_irq);

	/* .stack_fiq section */
	
	.stack_fiq (__stack_irq_end__ + 3) / 4 * 4 (NOLOAD) :
	{
		__stack_fiq_start__ = .;
		*(.stack_fiq)
	    . = MAX(__stack_fiq_start__ + _STACKSIZE_FIQ , .);
	} >DATA
	__stack_fiq_end__ = __stack_fiq_start__ + SIZEOF(.stack_fiq);

	/* .stack_svc section */
	
	.stack_svc (__stack_fiq_end__ + 3) / 4 * 4 (NOLOAD) :
	{
		__stack_svc_start__ = .;
		*(.stack_svc)
		. = MAX(__stack_svc_start__ + _STACKSIZE_SVC , .);
	} >DATA
	__stack_svc_end__ = __stack_svc_start__ + SIZEOF(.stack_svc);

	/* .stack_abt section */
	
	.stack_abt (__stack_svc_end__ + 3) / 4 * 4 (NOLOAD) :
	{
		__stack_abt_start__ = .;
		*(.stack_abt)
		. = MAX(__stack_abt_start__ + _STACKSIZE_ABT , .);
	} >DATA
	__stack_abt_end__ = __stack_abt_start__ + SIZEOF(.stack_abt);

	/* .stack_und section */
	
	.stack_und (__stack_abt_end__ + 3) / 4 * 4 (NOLOAD) :
	{
		__stack_und_start__ = .;
		*(.stack_und)
    	. = MAX(__stack_und_start__ + _STACKSIZE_UND , .);
	} >DATA
	__stack_und_end__ = __stack_und_start__ + SIZEOF(.stack_und);
  
	/* Stabs debugging sections.  */
	.stab          0 : { *(.stab) }
	.stabstr       0 : { *(.stabstr) }
	.stab.excl     0 : { *(.stab.excl) }
	.stab.exclstr  0 : { *(.stab.exclstr) }
	.stab.index    0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment       0 : { *(.comment) }
	/* DWARF debug sections.
		Symbols in the DWARF debugging sections are relative to the beginning
		of the section so we begin them at 0.  */
	/* DWARF 1 */
	.debug          0 : { *(.debug) }
	.line           0 : { *(.line) }
	/* GNU DWARF 1 extensions */
	.debug_srcinfo  0 : { *(.debug_srcinfo) }
	.debug_sfnames  0 : { *(.debug_sfnames) }
	/* DWARF 1.1 and DWARF 2 */
	.debug_aranges  0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	/* DWARF 2 */
	.debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
	.debug_abbrev   0 : { *(.debug_abbrev) }
	.debug_line     0 : { *(.debug_line) }
	.debug_frame    0 : { *(.debug_frame) }
	.debug_str      0 : { *(.debug_str) }
	.debug_loc      0 : { *(.debug_loc) }
	.debug_macinfo  0 : { *(.debug_macinfo) }
	/* SGI/MIPS DWARF 2 extensions */
	.debug_weaknames 0 : { *(.debug_weaknames) }
	.debug_funcnames 0 : { *(.debug_funcnames) }
	.debug_typenames 0 : { *(.debug_typenames) }
	.debug_varnames  0 : { *(.debug_varnames) }	
}
