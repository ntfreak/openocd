#
# Memory controller helpers
#

source [find target/mediatek/mmio.tcl]

proc ddr_init {cfgs dq_dly dqs_dly} {
    mmio_clrbits_32 0xb0000304 0x20000000
    mww 0xb0000348 [lindex $cfgs 2]
    mww 0xb000034c [lindex $cfgs 3]
    mww 0xb0000350 [lindex $cfgs 4]
    mww 0xb0000360 $dq_dly
    mww 0xb0000364 $dqs_dly
    mww 0xb0000340 [lindex $cfgs 0]
    mww 0xb0000344 [expr ([lindex $cfgs 1] & (~0x30000)) | 0x20000]
    mmio_clrsetbits_32 0xb0000314 0xffffff 1
    mmio_setbits_32 0xb0000310 0x10
}

proc sdr_init {cfgs rows cols} {
    mww 0xb0000300 [lindex $cfgs 0]
    mww 0xb0000304 [expr [lindex $cfgs 1] | (($rows & 3) << 16) | (($cols & 3) << 20)]

    set init_done 0
    while {$pll_locked == 0} {
        set init_done [expr [mmio_readfield_32 0xb0000304 30 1]]
    }

    mmio_clrsetbits_32 0xb0000314 0xffffff 1
    mmio_setbits_32 0xb0000310 0x10
}
