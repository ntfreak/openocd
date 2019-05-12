target remote localhost:3333
monitor reset
monitor sleep 500
monitor poll
monitor soft_reset_halt
monitor arm7_9 force_hw_bkpts enable
monitor mww 0xA0000050 0x01c2
monitor mdw 0xA0000050
load
break main
continue
