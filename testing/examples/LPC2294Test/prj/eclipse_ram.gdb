target remote localhost:3333
monitor reset
monitor sleep 500
monitor poll
monitor soft_reset_halt
monitor arm7_9 sw_bkpts enable
monitor mww 0xE01FC040 0x0002
monitor mdw 0xE01FC040
load
break main
continue
