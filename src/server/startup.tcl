# Defines basic Tcl procs for OpenOCD server modules

# Handle GDB 'R' packet. Can be overridden by configuration script,
# but it's not something one would expect target scripts to do
# normally
proc ocd_gdb_restart {target_id} {
	# Fix!!! we're resetting all targets here! Really we should reset only
	# one target
	reset halt
}

lappend _telnet_autocomplete_skip prevent_cps
lappend _telnet_autocomplete_skip POST
lappend _telnet_autocomplete_skip Host:
proc prevent_cps {} {
	echo "Possible SECURITY ATTACK detected."
	echo "It looks like somebody is sending POST or Host: commands to OpenOCD."
	echo "This is likely due to an attacker attempting to use Cross Protocol Scripting"
	echo "to compromise your OpenOCD instance. Connection aborted."
	exit
}

proc POST {args} { prevent_cps }
proc Host: {args} { prevent_cps }

# list of commands we don't want to appear in autocomplete
lappend _telnet_autocomplete_skip _telnet_autocomplete_helper

# helper for telnet autocomplete
proc _telnet_autocomplete_helper pattern {
	set cmds [info commands $pattern]

	# skip matches in variable '_telnet_autocomplete_skip'
	foreach skip $::_telnet_autocomplete_skip {
		foreach n [lsearch -all -regexp $cmds "^$skip\$"] {
			set cmds [lreplace $cmds $n $n]
		}
	}

	return [lsort $cmds]
}
