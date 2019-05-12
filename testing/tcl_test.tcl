if { $argc != 1 } {
	puts "Usage: test_tcl.tcl <ipaddress>"
	exit 1
}

puts $argv

# Simple tcl client to connect to openocd
global fo
set fo [socket $argv 6666]

# If a fn is unknown to Tcl, send it off to OpenOCD
proc unknown args {	
	global fo
	puts $fo $args
	flush $fo
	gets $fo line
	return $line
}



#Print help text for a command. Word wrap
#help text that is too wide inside column.
proc pc_help {args} {
	global ocd_helptext
	set cmd $args
	foreach a [lsort $ocd_helptext] {
		if {[string length $cmd]==0||[string first $cmd $a]!=-1||[string first $cmd [lindex $a 1]]!=-1} {
			set w 50
			set cmdname [lindex $a 0]
			set h [lindex $a 1]
			set n 0
			while 1 {
				if {$n > [string length $h]} {break}
				
				set next_a [expr $n+$w]
				if {[string length $h]>$n+$w} {
					set xxxx [string range $h $n [expr $n+$w]]
					for {set lastpos [expr [string length $xxxx]-1]} {$lastpos>=0&&[string compare [string range $xxxx $lastpos $lastpos] " "]!=0} {set lastpos [expr $lastpos-1]} {
					}
					#set next_a -1
					if {$lastpos!=-1} {
						set next_a [expr $lastpos+$n+1]
					}
				}
				
				
				puts [format "%-25s %s" $cmdname [string range $h $n [expr $next_a-1]] ]
				set cmdname ""
				set n [expr $next_a]
			}
		}
	}
}

puts "Running flash_banks"
puts [flash_banks]
puts "Running help on PC using data from OpenOCD"
global ocd_helptext
set ocd_helptext [get_help_text]
puts [pc_help]
