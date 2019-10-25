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

source [find cpu/arc/v2.tcl]

proc arc_em_examine_target { {target ""} } {
	# Will set current target
	arc_v2_examine_target $target
}

proc arc_em_init_regs { } {
	arc_v2_init_regs

	[target current] configure \
		-event examine-end "arc_em_examine_target [target current]"
}

# Scripts in "target" folder should call this function instead of direct
# invocation of arc_common_reset.
proc arc_em_reset { {target ""} } {
	arc_v2_reset $target

	# Set DEBUG.ED bit to enable clock in actionpoint module.
	# This is specific to ARC EM.
	set debug [arc jtag get-aux-reg 5]
	if { !($debug & (1 << 20)) } {
		arc jtag set-aux-reg 5 [expr $debug | (1 << 20)]
	}
}
