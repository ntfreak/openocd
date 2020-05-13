# Defines basic Tcl procs for OpenOCD target module

proc new_target_name { } {
	return [target number [expr [target count] - 1 ]]
}

global in_process_reset
set in_process_reset 0

# Catch reset recursion
proc ocd_process_reset { MODE } {
	global in_process_reset
	if {$in_process_reset} {
		set in_process_reset 0
		return -code error "'reset' can not be invoked recursively"
	}

	set in_process_reset 1
	set success [expr [catch {ocd_process_reset_inner $MODE} result]==0]
	set in_process_reset 0

	if {$success} {
		return $result
	} else {
		return -code error $result
	}
}

proc ocd_process_reset_inner { MODE } {
	set targets [target names]

	# If this target must be halted...
	set halt -1
	if { 0 == [string compare $MODE halt] } {
		set halt 1
	}
	if { 0 == [string compare $MODE init] } {
		set halt 1;
	}
	if { 0 == [string compare $MODE run ] } {
		set halt 0;
	}
	if { $halt < 0 } {
		return -code error "Invalid mode: $MODE, must be one of: halt, init, or run";
	}

	# Target event handlers *might* change which TAPs are enabled
	# or disabled, so we fire all of them.  But don't issue any
	# target "arp_*" commands, which may issue JTAG transactions,
	# unless we know the underlying TAP is active.
	#
	# NOTE:  ARP == "Advanced Reset Process" ... "advanced" is
	# relative to a previous restrictive scheme

	foreach t $targets {
		# New event script.
		$t invoke-event reset-start
	}

	# Use TRST or TMS/TCK operations to reset all the tap controllers.
	# TAP reset events get reported; they might enable some taps.
	init_reset $MODE

	# Examine all targets on enabled taps.
	foreach t $targets {
		if {![using_jtag] || [jtag tapisenabled [$t cget -chain-position]]} {
			$t invoke-event examine-start
			set err [catch "$t arp_examine allow-defer"]
			if { $err } {
				$t invoke-event examine-fail
			} else {
				$t invoke-event examine-end
			}
		}
	}

	# Assert SRST, and report the pre/post events.
	# Note:  no target sees SRST before "pre" or after "post".
	foreach t $targets {
		$t invoke-event reset-assert-pre
	}
	foreach t $targets {
		# C code needs to know if we expect to 'halt'
		if {![using_jtag] || [jtag tapisenabled [$t cget -chain-position]]} {
			$t arp_reset assert $halt
		}
	}
	foreach t $targets {
		$t invoke-event reset-assert-post
	}

	# Now de-assert SRST, and report the pre/post events.
	# Note:  no target sees !SRST before "pre" or after "post".
	foreach t $targets {
		$t invoke-event reset-deassert-pre
	}
	foreach t $targets {
		# Again, de-assert code needs to know if we 'halt'
		if {![using_jtag] || [jtag tapisenabled [$t cget -chain-position]]} {
			$t arp_reset deassert $halt
		}
	}
	foreach t $targets {
		$t invoke-event reset-deassert-post
	}

	# Pass 1 - Now wait for any halt (requested as part of reset
	# assert/deassert) to happen.  Ideally it takes effect without
	# first executing any instructions.
	if { $halt } {
		foreach t $targets {
			if {[using_jtag] && ![jtag tapisenabled [$t cget -chain-position]]} {
				continue
			}

			# don't wait for targets where examination is deferred
			# they can not be halted anyway at this point
			if { ![$t was_examined] && [$t examine_deferred] } {
				continue
			}

			# Wait up to 1 second for target to halt. Why 1sec? Cause
			# the JTAG tap reset signal might be hooked to a slow
			# resistor/capacitor circuit - and it might take a while
			# to charge

			# Catch, but ignore any errors.
			catch { $t arp_waitstate halted 1000 }

			# Did we succeed?
			set s [$t curstate]

			if { 0 != [string compare $s "halted" ] } {
				return -code error [format "TARGET: %s - Not halted" $t]
			}
		}
	}

	#Pass 2 - if needed "init"
	if { 0 == [string compare init $MODE] } {
		foreach t $targets {
			if {[using_jtag] && ![jtag tapisenabled [$t cget -chain-position]]} {
				continue
			}

			# don't wait for targets where examination is deferred
			# they can not be halted anyway at this point
			if { ![$t was_examined] && [$t examine_deferred] } {
				continue
			}

			set err [catch "$t arp_waitstate halted 5000"]
			# Did it halt?
			if { $err == 0 } {
				$t invoke-event reset-init
			}
		}
	}

	foreach t $targets {
		$t invoke-event reset-end
	}
}

proc using_jtag {} {
	set _TRANSPORT [ transport select ]
	expr { [ string first "jtag" $_TRANSPORT ] != -1 }
}

proc using_swd {} {
	set _TRANSPORT [ transport select ]
	expr { [ string first "swd" $_TRANSPORT ] != -1 }
}

proc using_hla {} {
	set _TRANSPORT [ transport select ]
	expr { [ string first "hla" $_TRANSPORT ] != -1 }
}

#########

# Target/chain configuration scripts can either execute commands directly
# or define a procedure which is executed once all configuration
# scripts have completed.
#
# By default(classic) the config scripts will set up the target configuration
proc init_targets {} {
}

proc set_default_target_event {t e s} {
	if {[$t cget -event $e] == ""} {
		$t configure -event $e $s
	}
}

proc init_target_events {} {
	set targets [target names]

	foreach t $targets {
		set_default_target_event $t gdb-flash-erase-start "reset init"
		set_default_target_event $t gdb-flash-write-end "reset halt"
		set_default_target_event $t gdb-attach "halt 1000"
	}
}

# Additionally board config scripts can define a procedure init_board that will be executed after init and init_targets
proc init_board {} {
}

# smp_on/smp_off were already DEPRECATED in v0.11.0 through http://openocd.zylin.com/4615
proc "aarch64 smp_on" {args} {
	echo "DEPRECATED! use 'aarch64 smp on' not 'aarch64 smp_on'"
	eval aarch64 smp on $args
}

proc "aarch64 smp_off" {args} {
	echo "DEPRECATED! use 'aarch64 smp off' not 'aarch64 smp_off'"
	eval aarch64 smp off $args
}

proc "cortex_a smp_on" {args} {
	echo "DEPRECATED! use 'cortex_a smp on' not 'cortex_a smp_on'"
	eval cortex_a smp on $args
}

proc "cortex_a smp_off" {args} {
	echo "DEPRECATED! use 'cortex_a smp off' not 'cortex_a smp_off'"
	eval cortex_a smp off $args
}

proc "mips_m4k smp_on" {args} {
	echo "DEPRECATED! use 'mips_m4k smp on' not 'mips_m4k smp_on'"
	eval mips_m4k smp on $args
}

proc "mips_m4k smp_off" {args} {
	echo "DEPRECATED! use 'mips_m4k smp off' not 'mips_m4k smp_off'"
	eval mips_m4k smp off $args
}
