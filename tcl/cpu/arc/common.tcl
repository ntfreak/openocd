#  Copyright (C) 2015 Synopsys, Inc.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the
#  Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

# Things common to all ARCs

# It is assumed that target is already halted.
proc arc_common_reset { {target ""} } {
    if { $target != "" } {
        targets $target
    }

    # This proc makes several operations and they should be transactional, so
    # we have to halt CPU, otherwise it is possible that we will, say, disable
    # interrupts, but then running CPU will enable them before we will move PC
    # to reset vector. So in theory we would need to halt and resume, then if
    # command was "reset halt" OpenOCD will halt the core. However that is
    # undesired because semantically "reset halt" should be halted on first
    # instruction after reset, and that will not happen if we will resume core
    # form this function. This event also doesn't have any arguments so we
    # cannot know if it was "reset halt" or not, so we cannot make "resume"
    # conditional. As a result this proc will always halt and user will always
    # have to resume... Not very user-friendly, sigh. But at least that seems
    # to work reliably though.
    halt

    # 1. Interrupts are disabled (STATUS32.IE)
    # 2. The status register flags are cleared.
    # All fields, except the H bit, are set to 0 when the processor is Reset.
    arc jtag set-aux-reg 0xA 0x1

    # 3. The loop count, loop start, and loop end registers are cleared.
    arc jtag set-core-reg 60 0
    arc jtag set-aux-reg 0x2 0
    arc jtag set-aux-reg 0x3 0

    # 4. The scoreboard unit is cleared.
    # 5. The pending-load flag is cleared.

    # Program execution begins at the address referenced by the four byte reset
    # vector located at the interrupt vector base address, which is the first
    # entry (offset 0x00) in the vector table.
    set int_vector_base [arc jtag get-aux-reg 0x25]
    set start_pc ""
    mem2array start_pc 32 $int_vector_base 1
    arc jtag set-aux-reg 0x6 $start_pc(0)

    # It is OK to do uncached writes - register cache will be invalidated by
    # the reset_assert() function.
}

# vim:expandtab:
