# Defines basic Tcl procs for OpenOCD flash module

#
# program utility proc
# usage: program filename
# optional args: verify, reset, exit and address
#

proc program_error {description exit} {
	if {$exit == 1} {
		echo $description
		shutdown error
	}

	error $description
}

proc program {filename args} {
	set exit 0
	set needsflash 1

	foreach arg $args {
		if {[string equal $arg "preverify"]} {
			set preverify 1
		} elseif {[string equal $arg "verify"]} {
			set verify 1
		} elseif {[string equal $arg "reset"]} {
			set reset 1
		} elseif {[string equal $arg "exit"]} {
			set exit 1
		} else {
			set address $arg
		}
	}

	# Set variables
	set filename \{$filename\}
	if {[info exists address]} {
		set flash_args "$filename $address"
	} else {
		set flash_args "$filename"
	}


	# make sure init is called
	if {[catch {init}] != 0} {
		program_error "** OpenOCD init failed **" 1
	}

	# reset target and call any init scripts
	if {[catch {reset init}] != 0} {
		program_error "** Unable to reset target **" $exit
	}

	# Check whether programming is needed
	if {[info exists preverify]} {
		echo "**pre-verifying**"
		if {[catch {eval verify_image $flash_args}] == 0} {
			echo "**Verified OK - No flashing**"
			set needsflash 0
		}
	}

	# start programming phase
	if {$needsflash == 1} {
		echo "** Programming Started **"

		if {[catch {eval flash write_image erase $flash_args}] == 0} {
			echo "** Programming Finished **"
			if {[info exists verify]} {
				# verify phase
				echo "** Verify Started **"
				if {[catch {eval verify_image $flash_args}] == 0} {
					echo "** Verified OK **"
				} else {
					program_error "** Verify Failed **" $exit
				}
			}
		} else {
			program_error "** Programming Failed **" $exit
		}
	}

	if {[info exists reset]} {
		# reset target if requested
		if {$exit == 1} {
			# also disable target polling, we are shutting down anyway
			poll off
		}
		echo "** Resetting Target **"
		reset run
	}


	if {$exit == 1} {
		shutdown
	}
	return
}

add_help_text program "write an image to flash, address is only required for binary images. verify, reset, exit are optional"
add_usage_text program "<filename> \[address\] \[pre-verify\] \[verify\] \[reset\] \[exit\]"

# stm32[f0x|f3x] uses the same flash driver as the stm32f1x
proc stm32f0x args { eval stm32f1x $args }
proc stm32f3x args { eval stm32f1x $args }

# stm32[f4x|f7x] uses the same flash driver as the stm32f2x
proc stm32f4x args { eval stm32f2x $args }
proc stm32f7x args { eval stm32f2x $args }

# stm32lx driver supports both STM32 L0 and L1 devices
proc stm32l0x args { eval stm32lx $args }
proc stm32l1x args { eval stm32lx $args }

# stm32[g0|g4|wb|wl] uses the same flash driver as the stm32l4x
proc stm32g0x args { eval stm32l4x $args }
proc stm32g4x args { eval stm32l4x $args }
proc stm32l5x args { eval stm32l4x $args }
proc stm32wbx args { eval stm32l4x $args }
proc stm32wlx args { eval stm32l4x $args }

# ease migration to updated flash driver
proc stm32x args {
	echo "DEPRECATED! use 'stm32f1x $args' not 'stm32x $args'"
	eval stm32f1x $args
}

proc stm32f2xxx args {
	echo "DEPRECATED! use 'stm32f2x $args' not 'stm32f2xxx $args'"
	eval stm32f2x $args
}
