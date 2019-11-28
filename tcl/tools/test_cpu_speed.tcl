# Description:
#  Measure the CPU clock frequency of an ARM Cortex-M based device.
#
# Return:
#  The CPU clock frequency in Hz.
#
# Note:
#  You may need to adapt the number of cycles for your device.
#
proc cortex_m_test_cpu_speed { address { timeout 200 } { cycles_per_loop 4 } } {
	set loop_counter_start 0xffffffff

	halt

	# We place the following code at the given address to measure the
	# CPU clock frequency:
	#
	# 3801: subs r0, #1
	# d1fd: bne #-2
	# e7fe: b #-4
	array set loop_code {0 0xd1fd3801 1 0x0000e7fe}
	array2mem loop_code 32 $address 2

	reg pc $address
	reg r0 $loop_counter_start
	resume
	sleep $timeout
	halt

	set loop_counter_end [string range [reg r0] 10 end]
	set loop_counter_diff [expr $loop_counter_start - $loop_counter_end]

	return [expr double($loop_counter_diff) * $cycles_per_loop / $timeout * 1000 ]
}
