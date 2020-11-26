/* Application memory map */
MEMORY {
    CODE_RAM (RWX)  : ORIGIN = 0x200C0000, LENGTH = 0x1000

	/* g_cfg + g_buf => 16 + 4096 */
    DATA_RAM (RWX)  : ORIGIN = 0x200C1000, LENGTH = 0x1010
}

/* Sections used for flashing */
SECTIONS
{
	.text :
	{
		_text = .;
		*(.entry*)
		*(.text*)
		_etext = .;
	} > CODE_RAM

	.data :
	{	_data = .;
		*(.rodata*)
		*(.data*)
		_edata = .;
	}

	.bss :
	{
		__bss_start__ = .;
		_bss = .;
		*(.bss*)
		*(COMMON)
		_ebss = .;
		__bss_end__ = .;
	} > CODE_RAM

	.stack :
	{
		_stack = .;
		*(.stack*)
		_estack = .;
	} > CODE_RAM

	.buffers :
	{
		_buffers = .;
		*(.buffers.g_cfg)
		*(.buffers.g_buf)
		*(.buffers*)
		_ebuffers = .;
	} > DATA_RAM
}
