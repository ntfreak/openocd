/* SPDX-License-Identifier: GPL-2.0+ */
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
	. = 0x0;
	. = ALIGN(4);
	.text : {
		sh_qspi.o (.text*)
		*(.text*)
	}
}
