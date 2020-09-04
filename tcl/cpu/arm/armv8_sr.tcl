# ARMv8 Architected System Register support
#
# Copyright (c) 2020-2021, Ampere Computing LLC
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program;
#
#

# Command Usage
#
# Execute TCL script to load the read_reg and write_reg routines
# Example: script ./tcl/cpu/arm/armv8_sr.tcl
#
# read_reg target_id sys_reg_name ['sec' | 'nsec' | 'asis']
#    target_id    : OpenOCD ARMv8 core target
#    sys_reg_name : One of the reg names in the get_sr_encoding routine
#    'sec'  : perform system reg read at highest exception level and ns=0
#    'nsec' : perform system reg read at highest exception level and ns=1
#    'asis' : perform sys reg read at current exception level and ns setting
#
# write_reg target_id sys_reg_name write_value ['sec' | 'nsec' | 'asis']
#    target_id    : OpenOCD ARMv8 core target
#    sys_reg_name : One of the reg names in the get_sr_encoding routine
#    write_value  : The numeric value in hexadecimal or decimal
#    'sec'  : perform system reg write at highest exception level and ns=0
#    'nsec' : perform system reg write at highest exception level and ns=1
#    'asis' : perform sys reg write at current exception level and ns setting

proc get_sr_encoding {sys_reg_name} {
	set sr_encoding {}

	set sys_reg_name [string toupper $sys_reg_name]

	switch $sys_reg_name {
		SCTLR_EL3 {set sr_encoding {3 6 1 0 0}}
		SCR_EL3 {set sr_encoding {3 6 1 1 0}}
		CPTR_EL3 {set sr_encoding {3 6 1 1 2}}
		SP_EL2 {set sr_encoding {3 6 4 1 0}}
		VBAR_EL3 {set sr_encoding {3 6 12 0 0}}
		ICC_CTLR_EL3 {set sr_encoding {3 6 12 12 4}}
		ICC_SRE_EL3 {set sr_encoding {3 6 12 12 5}}
		ICC_IGRPEN1_EL3 {set sr_encoding {3 6 12 12 7}}
		MPIDR_EL1 {set sr_encoding {3 0 0 0 5}}
		SCTLR_EL1 {set sr_encoding {3 0 1 0 0}}
		CPACR_EL1 {set sr_encoding {3 0 1 0 2}}
		ELR_EL1 {set sr_encoding {3 0 4 0 1}}
		SP_EL0 {set sr_encoding {3 0 4 1 0}}
		SPSEL {set sr_encoding {3 0 4 2 0}}
		CURRENTEL {set sr_encoding {3 0 4 2 2}}
		ESR_EL1 {set sr_encoding {3 0 5 2 0}}
		ERRIDR_EL1 {set sr_encoding {3 0 5 3 0}}
		ERRSELR_EL1 {set sr_encoding {3 0 5 3 1}}
		ERXFR_EL1 {set sr_encoding {3 0 5 4 0}}
		ERXCTLR_EL1 {set sr_encoding {3 0 5 4 1}}
		ERXSTATUS_EL1 {set sr_encoding {3 0 5 4 2}}
		ERXADDR_EL1 {set sr_encoding {3 0 5 4 3}}
		ERXMISC0_EL1 {set sr_encoding {3 0 5 5 0}}
		ERXMISC1_EL1 {set sr_encoding {3 0 5 5 1}}
		FAR_EL1 {set sr_encoding {3 0 6 0 0}}
		PAR_EL1 {set sr_encoding {3 0 7 4 0}}
		VBAR_EL1 {set sr_encoding {3 0 12 0 0}}
		ICC_IAR0_EL1 {set sr_encoding {3 0 12 8 0}}
		ICC_EOIR0_EL1 {set sr_encoding {3 0 12 8 1}}
		ICC_HPPIR0_EL1 {set sr_encoding {3 0 12 8 2}}
		ICC_DIR_EL1 {set sr_encoding {3 0 12 11 1}}
		ICC_RPR_EL1 {set sr_encoding {3 0 12 11 3}}
		ICC_HPPIR1_EL1 {set sr_encoding {3 0 12 12 2}}
		ICC_CTLR_EL1 {set sr_encoding {3 0 12 12 4}}
		ICC_SRE_EL1 {set sr_encoding {3 0 12 12 5}}
		ICC_IGRPEN0_EL1 {set sr_encoding {3 0 12 12 6}}
		ICC_IGRPEN1_EL1 {set sr_encoding {3 0 12 12 7}}
		ICC_IAR1_EL1 {set sr_encoding {3 0 12 12 0}}
		ERXPFGF_EL1 {set sr_encoding {3 0 5 4 4}}
		ERXPFGCTL_EL1 {set sr_encoding {3 0 5 4 5}}
		ERXPFGCDN_EL1 {set sr_encoding {3 0 5 4 6}}
		CNTPCT_EL0 {set sr_encoding {3 3 14 0 1}}
		CNTP_TVAL_EL0 {set sr_encoding {3 3 14 2 0}}
		CNTP_CVAL_EL0 {set sr_encoding {3 3 14 2 2}}
		SCTLR_EL2 {set sr_encoding {3 4 1 0 0}}
		HCR_EL2 {set sr_encoding {3 4 1 1 0}}
		CPTR_EL2 {set sr_encoding {3 4 1 1 2}}
		SP_EL1 {set sr_encoding {3 4 4 1 0}}
		CNTPS_TVAL_EL1 {set sr_encoding {3 7 14 2 0}}
		CNTPS_CTL_EL1 {set sr_encoding {3 7 14 2 1}}
		CNTPS_CVAL_EL1 {set sr_encoding {3 7 14 2 2}}
		default {set sr_encoding {"Unsupported system register name"}}
	}

	return $sr_encoding
}

proc read_reg_usage {} {
	echo {read_reg target_id sys_reg_name ['sec' | 'nsec' | 'asis']}
	echo {   target_id    : OpenOCD ARMv8 core target identifier}
	echo {   sys_reg_name : a supported system register}
	echo {   'sec'  : read at highest exception level and ns=0}
	echo {   'nsec' : read at highest exception level and ns=1}
	echo {   'asis' : read at current exception level and ns setting}
	echo {}
}

proc read_reg {args} {
	set num [llength $args]
	if { $num == 2 } {
		set target_id [lindex $args 0]
		set sys_reg_name [lindex $args 1]
	} elseif { $num == 3 } {
		set target_id [lindex $args 0]
		set sys_reg_name [lindex $args 1]
		set option [string tolower [lindex $args 2]]
		if { $option != {sec} && $option != {nsec} && $option != {asis} } {
			read_reg_usage
			return {Invalid read_reg parameters}
		}
	} else {
		read_reg_usage
		return {Invalid read_reg parameters}
	}

	set sr_enc [get_sr_encoding $sys_reg_name]

	if {[llength $sr_enc] == 5 } {
		if {$num == 2} {
			set result [$target_id aarch64 mrs [lindex $sr_enc 0] [lindex $sr_enc 1] [lindex $sr_enc 2] [lindex $sr_enc 3] [lindex $sr_enc 4]]
		} else {
			set result [$target_id aarch64 mrs $option [lindex $sr_enc 0] [lindex $sr_enc 1] [lindex $sr_enc 2] [lindex $sr_enc 3] [lindex $sr_enc 4]]
		}
	} else {
		set result "Unsupported system register name $sys_reg_name"
		read_reg_usage
	}

	return $result
}

proc write_reg_usage {} {
	echo {write_reg target_id sys_reg_name write_value ['sec' | 'nsec' | 'asis']}
	echo {   target_id    : OpenOCD ARMv8 core target identifier}
	echo {   sys_reg_name : a supported system register}
	echo {   write_value  : write value in hexadecimal or decimal}
	echo {   'sec'  : write at highest exception level and ns=0}
	echo {   'nsec' : write at highest exception level and ns=1}
	echo {   'asis' : write at current exception level and ns setting}
	echo {}
}

proc write_reg {args} {
	set num [llength $args]
	if { $num == 3 } {
		set target_id [lindex $args 0]
		set sys_reg_name [lindex $args 1]
		set write_value [lindex $args 2]
	} elseif { $num == 4 } {
		set target_id [lindex $args 0]
		set sys_reg_name [lindex $args 1]
		set write_value [lindex $args 2]
		set option [string tolower [lindex $args 3]]
		if { $option != {sec} && $option != {nsec} && $option != {asis} } {
			write_reg_usage
			return {Invalid write_reg parameters}
		}
	} else {
		write_reg_usage
		return {Invalid write_reg parameters}
	}

	set sr_enc [get_sr_encoding $sys_reg_name]

	if {[llength $sr_enc] == 5} {
		if {$num == 3} {
			set result [$target_id aarch64 msr [lindex $sr_enc 0] [lindex $sr_enc 1] [lindex $sr_enc 2] [lindex $sr_enc 3] [lindex $sr_enc 4] $write_value]
		} else {
			set result [$target_id aarch64 msr $option [lindex $sr_enc 0] [lindex $sr_enc 1] [lindex $sr_enc 2] [lindex $sr_enc 3] [lindex $sr_enc 4] $write_value]
		}
	} else {
		set result "Unsupported system register name $sys_reg_name"
		write_reg_usage
	}

	return $result
}

echo "Added Commands:"
read_reg_usage
write_reg_usage
