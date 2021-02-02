#
# MMIO register r/w helpers
#

proc mmio_read_32 {addr} {
    set result ""
    mem2array result 32 $addr 1
    return $result(0)
}

proc mmio_clrbits_32 {addr bits} {
    mww $addr [expr [mmio_read_32 $addr] & (~($bits))]
}

proc mmio_setbits_32 {addr bits} {
    mww $addr [expr [mmio_read_32 $addr] | $bits]
}

proc mmio_clrsetbits_32 {addr clr set} {
    mww $addr [expr ([mmio_read_32 $addr] & (~($clr))) | $set]
}

proc mmio_readfield_32 {addr shift mask} {
    return [expr ([mmio_read_32 $addr] >> $shift) & $mask]
}
