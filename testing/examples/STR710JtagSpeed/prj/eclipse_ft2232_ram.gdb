target remote localhost:3333

monitor reset
monitor sleep 500
monitor poll
monitor soft_reset_halt
monitor arm7_9 sw_bkpts enable
monitor mww 0xA0000050 0x01c2
monitor mdw 0xA0000050
monitor mww 0x6C000004 0x8005
monitor mdw 0x6C000004
monitor mww 0xE0005000 0xFFFF
monitor mww 0xE0005004 0x00FF
monitor mww 0xE0005008 0xFFFF
monitor mdw 0xE0005000
monitor mdw 0xE0005004
monitor mdw 0xE0005008
monitor mww 0xE000500C 0x0000

monitor arm7_9 fast_memory_access enable
monitor arm7_9 dcc_downloads enable
monitor verify_ircapture disable

load
break main
continue
