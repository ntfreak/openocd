# Defines basic Tcl procs for OpenOCD JTAG module

# Executed during "init". Can be overridden
# by board/target/... scripts
proc jtag_init {} {
	if {[catch {jtag arp_init} err]!=0} {
		# try resetting additionally
		init_reset startup
	}
}

# This reset logic may be overridden by board/target/... scripts as needed
# to provide a reset that, if possible, is close to a power-up reset.
#
# Exit requirements include:  (a) JTAG must be working, (b) the scan
# chain was validated with "jtag arp_init" (or equivalent), (c) nothing
# stays in reset.  No TAP-specific scans were performed.  It's OK if
# some targets haven't been reset yet; they may need TAP-specific scans.
#
# The "mode" values include:  halt, init, run (from "reset" command);
# startup (at OpenOCD server startup, when JTAG may not yet work); and
# potentially more (for reset types like cold, warm, etc)
proc init_reset { mode } {
	if {[using_jtag]} {
		jtag arp_init-reset
	}
}

#########

# TODO: power_restore and power_dropout are currently neither
# documented nor supported except on ZY1000.

proc power_restore {} {
	echo "Sensed power restore, running reset init and halting GDB."
	reset init

	# Halt GDB so user can deal with a detected power restore.
	#
	# After GDB is halted, then output is no longer forwarded
	# to the GDB console.
	set targets [target names]
	foreach t $targets {
		# New event script.
		$t invoke-event arp_halt_gdb
	}
}

add_help_text power_restore "Overridable procedure run when power restore is detected. Runs 'reset init' by default."

proc power_dropout {} {
	echo "Sensed power dropout."
}

#########

# TODO: srst_deasserted and srst_asserted are currently neither
# documented nor supported except on ZY1000.

proc srst_deasserted {} {
	echo "Sensed nSRST deasserted, running reset init and halting GDB."
	reset init

	# Halt GDB so user can deal with a detected reset.
	#
	# After GDB is halted, then output is no longer forwarded
	# to the GDB console.
	set targets [target names]
	foreach t $targets {
		# New event script.
		$t invoke-event arp_halt_gdb
	}
}

add_help_text srst_deasserted "Overridable procedure run when srst deassert is detected. Runs 'reset init' by default."

proc srst_asserted {} {
	echo "Sensed nSRST asserted."
}

# measure actual JTAG clock
proc measure_clk {} {
	set start_time [ms];
        set iterations 10000000;
	runtest $iterations;
	echo "Running at more than [expr $iterations.0 / ([ms]-$start_time)] kHz";
}

add_help_text measure_clk "Runs a test to measure the JTAG clk. Useful with RCLK / RTCK."

proc default_to_jtag { f args } {
	set current_transport [transport select]
	if {[using_jtag]} {
		eval $f $args
	} {
		error "session transport is \"$current_transport\" but your config requires JTAG"
	}
}

proc jtag args {
	eval default_to_jtag jtag $args
}

proc jtag_rclk args {
	eval default_to_jtag jtag_rclk $args
}

proc jtag_ntrst_delay args {
	eval default_to_jtag jtag_ntrst_delay $args
}

proc jtag_ntrst_assert_width args {
	eval default_to_jtag jtag_ntrst_assert_width $args
}

# BEGIN MIGRATION AIDS ...  these adapter operations originally had
# JTAG-specific names despite the fact that the operations were not
# specific to JTAG, or otherwise had troublesome/misleading names.
#
# FIXME phase these aids out after about April 2011
#
proc jtag_khz args {
	echo "DEPRECATED! use 'adapter speed' not 'jtag_khz'"
	eval adapter speed $args
}

proc jtag_nsrst_delay args {
	echo "DEPRECATED! use 'adapter srst delay' not 'jtag_nsrst_delay'"
	eval adapter srst delay $args
}

proc jtag_nsrst_assert_width args {
	echo "DEPRECATED! use 'adapter srst pulse_width' not 'jtag_nsrst_assert_width'"
	eval adapter srst pulse_width $args
}

proc jtag_reset args {
	echo "DEPRECATED! use 'adapter \[de\]assert' not 'jtag_reset'"
	switch $args {
		"0 0"
			{eval adapter deassert trst deassert srst}
		"0 1"
			{eval adapter deassert trst assert srst}
		"1 0"
			{eval adapter assert trst deassert srst}
		"1 1"
			{eval adapter assert trst assert srst}
		default
			{return -code 1 -level 1 "jtag_reset: syntax error"}
	}
}

# stlink migration helpers
proc stlink_device_desc args {
	echo "DEPRECATED! use 'hla_device_desc' not 'stlink_device_desc'"
	eval hla_device_desc $args
}

proc stlink_serial args {
	echo "DEPRECATED! use 'hla_serial' not 'stlink_serial'"
	eval hla_serial $args
}

proc stlink_layout args {
	echo "DEPRECATED! use 'hla_layout' not 'stlink_layout'"
	eval hla_layout $args
}

proc stlink_vid_pid args {
	echo "DEPRECATED! use 'hla_vid_pid' not 'stlink_vid_pid'"
	eval hla_vid_pid $args
}

proc stlink args {
	echo "DEPRECATED! use 'hla' not 'stlink'"
	eval hla $args
}

proc adapter_khz args {
	echo "DEPRECATED! use 'adapter speed' not 'adapter_khz'"
	eval adapter speed $args
}

proc adapter_name args {
	echo "DEPRECATED! use 'adapter name' not 'adapter_name'"
	eval adapter name $args
}

proc adapter_nsrst_delay args {
	echo "DEPRECATED! use 'adapter srst delay' not 'adapter_nsrst_delay'"
	eval adapter srst delay $args
}

proc adapter_nsrst_assert_width args {
	echo "DEPRECATED! use 'adapter srst pulse_width' not 'adapter_nsrst_assert_width'"
	eval adapter srst pulse_width $args
}

proc interface args {
	echo "DEPRECATED! use 'adapter driver' not 'interface'"
	eval adapter driver $args
}

proc  interface_transports args {
	echo "DEPRECATED! use 'adapter transports' not 'interface_transports'"
	eval adapter transports $args
}

proc  interface_list args {
	echo "DEPRECATED! use 'adapter list' not 'interface_list'"
	eval adapter list $args
}

proc ftdi_location args {
	echo "DEPRECATED! use 'adapter usb location' not 'ftdi_location'"
	eval adapter usb location $args
}

proc xds110_serial args {
	echo "DEPRECATED! use 'xds110 serial' not 'xds110_serial'"
	eval xds110 serial $args
}

proc xds110_supply_voltage args {
	echo "DEPRECATED! use 'xds110 supply' not 'xds110_supply_voltage'"
	eval xds110 supply $args
}

proc hla {cmd args} {
        tailcall "hla $cmd" {*}$args
}

proc "hla newtap" {args} {
	echo "DEPRECATED! use 'swj_newdap' not 'hla newtap'"
	eval swj_newdap $args
}

# END MIGRATION AIDS
