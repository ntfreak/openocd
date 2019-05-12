target remote localhost:3333
monitor reset
monitor sleep 500
monitor poll
monitor soft_reset_halt
monitor arm7_9 force_hw_bkpts enable

# Set SRAM size to 96 KB
monitor mww 0x5C002034 0x0197
monitor mdw 0x5C002034

# Set Flash, Bank0 size to 512 KB
monitor mww 0x54000000 0xf

load
break main
continue
