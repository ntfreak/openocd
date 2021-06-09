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
lappend _telnet_autocomplete_skip _telnet_cmd_autocomplete_helper
lappend _telnet_autocomplete_skip _telnet_path_autocomplete_helper
lappend _telnet_autocomplete_skip _telnet_autocomplete_helper

proc _telnet_cmd_autocomplete_helper pattern {
	set cmds [info commands $pattern]

	# skip matches in variable '_telnet_autocomplete_skip'
	foreach skip $::_telnet_autocomplete_skip {
		foreach n [lsearch -all -regexp $cmds "^$skip\$"] {
			set cmds [lreplace $cmds $n $n]
		}
	}

	return [lsort $cmds]
}

proc _telnet_path_autocomplete_helper pattern {
	set DEBUG 1

	# TODO: IMPORTANT: quoted paths are not supported correctly

	# get the last argument
	# FIXME linux accepts double quotes in file names

	set escape false

	if {[regexp -indices {["'][^"']+\*$} $pattern indices]} {
		# this accepts quoted paths
		if { $DEBUG } { echo "quoted path detected" }
		set offset [expr [lindex $indices 0] + 1]
	} elseif {[regexp -indices {(\\ |\S)+\*$} $pattern indices]} {
		# this accepts paths with escaped whitespaces ('\ ')
		if { $DEBUG } { echo "path with escaped whitespaces detected" }
		set escape 1
		set offset [lindex $indices 0]
	} elseif {[regexp -indices {(\S)+\*$} $pattern indices]} {
		# this accepts paths without whitespace
		if { $DEBUG } { echo "path without whitespaces detected" }
		set offset [lindex $indices 0]
	} else {
		if { $DEBUG } { echo "no pattern detected" }
		# do not complain, just return an empty list
		return {}
	}

	set path_prefix [string range $pattern 0 [expr $offset - 1]]
	set path_pattern [string range $pattern $offset end]

	if { $DEBUG } { echo "path prefix: '$path_prefix'" }
	if { $DEBUG } { echo "path pattern: '$path_pattern'" }

	# TODO check windows support

	set paths [glob -nocomplain $path_pattern]

	# if it is a directorry append '/'
	foreach path $paths {
		if {[file isdirectory $path]} {
			append path /
		}

		if {$escape} {
			set path [regsub -all {\s} $path "\\ "]
		}

		lappend completions $path_prefix$path
	}

	return [lsort $completions]
}

# helper for telnet autocomplete
proc _telnet_autocomplete_helper pattern {
	set cmds [_telnet_cmd_autocomplete_helper $pattern]

	if {[llength $cmds] > 0} {
		return $cmds
	}

	return [_telnet_path_autocomplete_helper $pattern]
}
