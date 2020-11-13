/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_INTERFACE_H
#define OPENOCD_JTAG_INTERFACE_H

#include <jtag/jtag.h>
#include <jtag/swim.h>
#include <target/arm_tpiu_swo.h>

/* @file
 * The "Cable Helper API" is what the cable drivers can use to help
 * implement their "Cable API".  So a Cable Helper API is a set of
 * helper functions used by cable drivers, and this is different from a
 * Cable API.  A "Cable API" is what higher level code used to talk to a
 * cable.
 */


/** implementation of wrapper function tap_set_state() */
void tap_set_state_impl(tap_state_t new_state);

/**
 * This function sets the state of a "state follower" which tracks the
 * state of the TAPs connected to the cable.  The state follower is
 * hopefully always in the same state as the actual TAPs in the jtag
 * chain, and will be so if there are no bugs in the tracking logic
 * within that cable driver.
 *
 * All the cable drivers call this function to indicate the state they
 * think the TAPs attached to their cables are in.  Because this
 * function can also log transitions, it will be helpful to call this
 * function with every transition that the TAPs being manipulated are
 * expected to traverse, not just end points of a multi-step state path.
 *
 * @param new_state The state we think the TAPs are currently in (or
 * are about to enter).
 */
#define tap_set_state(new_state) \
	do { \
		LOG_DEBUG_IO("tap_set_state(%s)", tap_state_name(new_state)); \
		tap_set_state_impl(new_state); \
	} while (0)

/**
 * This function gets the state of the "state follower" which tracks the
 * state of the TAPs connected to the cable. @see tap_set_state @return
 * tap_state_t The state the TAPs are in now.
 */
tap_state_t tap_get_state(void);

/**
 * This function sets the state of an "end state follower" which tracks
 * the state that any cable driver thinks will be the end (resultant)
 * state of the current TAP SIR or SDR operation.
 *
 * At completion of that TAP operation this value is copied into the
 * state follower via tap_set_state().
 *
 * @param new_end_state The state the TAPs should enter at completion of
 * a pending TAP operation.
 */
void tap_set_end_state(tap_state_t new_end_state);

/**
 * For more information, @see tap_set_end_state
 * @return tap_state_t - The state the TAPs should be in at completion of the current TAP operation.
 */
tap_state_t tap_get_end_state(void);

/**
 * This function provides a "bit sequence" indicating what has to be
 * done with TMS during a sequence of seven TAP clock cycles in order to
 * get from state \a "from" to state \a "to".
 *
 * The length of the sequence must be determined with a parallel call to
 * tap_get_tms_path_len().
 *
 * @param from The starting state.
 * @param to The desired final state.
 * @return int The required TMS bit sequence, with the first bit in the
 * sequence at bit 0.
 */
int tap_get_tms_path(tap_state_t from, tap_state_t to);

/**
 * Function int tap_get_tms_path_len
 * returns the total number of bits that represents a TMS path
 * transition as given by the function tap_get_tms_path().
 *
 * For at least one interface (JLink) it's not OK to simply "pad" TMS
 * sequences to fit a whole byte.  (I suspect this is a general TAP
 * problem within OOCD.) Padding TMS causes all manner of instability
 * that's not easily discovered.  Using this routine we can apply
 * EXACTLY the state transitions required to make something work - no
 * more - no less.
 *
 * @param from is the starting state
 * @param to is the resultant or final state
 * @return int - the total number of bits in a transition.
 */
int tap_get_tms_path_len(tap_state_t from, tap_state_t to);


/**
 * Function tap_move_ndx
 * when given a stable state, returns an index from 0-5.  The index corresponds to a
 * sequence of stable states which are given in this order: <p>
 * { TAP_RESET, TAP_IDLE, TAP_DRSHIFT, TAP_DRPAUSE, TAP_IRSHIFT, TAP_IRPAUSE }
 * <p>
 * This sequence corresponds to look up tables which are used in some of the
 * cable drivers.
 * @param astate is the stable state to find in the sequence.  If a non stable
 *  state is passed, this may cause the program to output an error message
 *  and terminate.
 * @return int - the array (or sequence) index as described above
 */
int tap_move_ndx(tap_state_t astate);

/**
 * Function tap_is_state_stable
 * returns true if the \a astate is stable.
 */
bool tap_is_state_stable(tap_state_t astate);

/**
 * Function tap_state_transition
 * takes a current TAP state and returns the next state according to the tms value.
 * @param current_state is the state of a TAP currently.
 * @param tms is either zero or non-zero, just like a real TMS line in a jtag interface.
 * @return tap_state_t - the next state a TAP would enter.
 */
tap_state_t tap_state_transition(tap_state_t current_state, bool tms);

/** Allow switching between old and new TMS tables. @see tap_get_tms_path */
void tap_use_new_tms_table(bool use_new);
/** @returns True if new TMS table is active; false otherwise. */
bool tap_uses_new_tms_table(void);


/**
 * @brief Prints verbose TAP state transitions for the given TMS/TDI buffers.
 * @param tms_buf must points to a buffer containing the TMS bitstream.
 * @param tdi_buf must points to a buffer containing the TDI bitstream.
 * @param tap_len must specify the length of the TMS/TDI bitstreams.
 * @param start_tap_state must specify the current TAP state.
 * @returns the final TAP state; pass as @a start_tap_state in following call.
 */
static inline tap_state_t jtag_debug_state_machine(const void *tms_buf,
		const void *tdi_buf, unsigned tap_len, tap_state_t start_tap_state)
{
	/* Private declaration */
	tap_state_t jtag_debug_state_machine_(const void *tms_buf, const void *tdi_buf,
			unsigned tap_len, tap_state_t start_tap_state);

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO))
		return jtag_debug_state_machine_(tms_buf, tdi_buf, tap_len, start_tap_state);
	else
		return start_tap_state;
}

/**
 * Represents a driver for a debugging interface.
 *
 * @todo Rename; perhaps "debug_driver".  This isn't an interface,
 * it's a driver!  Also, not all drivers support JTAG.
 *
 * @todo We need a per-instance structure too, and changes to pass
 * that structure to the driver.  Instances can for example be in
 * either SWD or JTAG modes.  This will help remove globals, and
 * eventually to cope with systems which have more than one such
 * debugging interface.
 */
struct jtag_interface {
	/**
	 * Bit vector listing capabilities exposed by this driver.
	 */
	unsigned supported;
#define DEBUG_CAP_TMS_SEQ	(1 << 0)

	/**
	 * Execute queued commands.
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*execute_queue)(void);
};

/**
 * Represents a driver for a debugging interface
 *
 * @todo We need a per-instance structure too, and changes to pass
 * that structure to the driver.  Instances can for example be in
 * either SWD or JTAG modes.  This will help remove globals, and
 * eventually to cope with systems which have more than one such
 * debugging interface.
 */
struct adapter_driver {
	/** The name of the interface driver. */
	const char * const name;

	/** transports supported in C code (NULL terminated vector) */
	const char * const *transports;

	/**
	 * The interface driver may register additional commands to expose
	 * additional features not covered by the standard command set.
	 */
	const struct command_registration *commands;

	/**
	 * Interface driver must initialize any resources and connect to a
	 * JTAG device.
	 *
	 * quit() is invoked if and only if init() succeeds. quit() is always
	 * invoked if init() succeeds. Same as malloc() + free(). Always
	 * invoke free() if malloc() succeeds and do not invoke free()
	 * otherwise.
	 *
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*init)(void);

	/**
	 * Interface driver must tear down all resources and disconnect from
	 * the JTAG device.
	 *
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*quit)(void);

	/**
	 * Control (assert/deassert) the signals SRST and TRST on the interface.
	 * This function is synchronous and should be called after the adapter
	 * queue has been properly flushed.
	 * This function is optional.
	 * Adapters that don't support resets can either not define this function
	 * or return an error code.
	 * Adapters that don't support one of the two reset should ignore the
	 * request to assert the missing signal and eventually log an error.
	 *
	 * @param srst 1 to assert SRST, 0 to deassert SRST.
	 * @param trst 1 to assert TRST, 0 to deassert TRST.
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*reset)(int srst, int trst);

	/**
	 * Set the interface speed.
	 * @param speed The new interface speed setting.
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*speed)(int speed);

	/**
	 * Returns JTAG maximum speed for KHz. 0 = RTCK. The function returns
	 *  a failure if it can't support the KHz/RTCK.
	 *
	 *  WARNING!!!! if RTCK is *slow* then think carefully about
	 *  whether you actually want to support this in the driver.
	 *  Many target scripts are written to handle the absence of RTCK
	 *  and use a fallback kHz TCK.
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*khz)(int khz, int *jtag_speed);

	/**
	 * Calculate the clock frequency (in KHz) for the given @a speed.
	 * @param speed The desired interface speed setting.
	 * @param khz On return, contains the speed in KHz (0 for RTCK).
	 * @returns ERROR_OK on success, or an error code if the
	 * interface cannot support the specified speed (KHz or RTCK).
	 */
	int (*speed_div)(int speed, int *khz);

	/**
	 * Read and clear the power dropout flag. Note that a power dropout
	 * can be transitionary, easily much less than a ms.
	 *
	 * To find out if the power is *currently* on, one must invoke this
	 * method twice.  Once to clear the power dropout flag and a second
	 * time to read the current state.  The default implementation
	 * never reports power dropouts.
	 *
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*power_dropout)(int *power_dropout);

	/**
	 * Read and clear the srst asserted detection flag.
	 *
	 * Like power_dropout this does *not* read the current
	 * state.  SRST assertion is transitionary and may be much
	 * less than 1ms, so the interface driver must watch for these
	 * events until this routine is called.
	 *
	 * @param srst_asserted On return, indicates whether SRST has
	 * been asserted.
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*srst_asserted)(int *srst_asserted);

	/**
	 * Configure trace parameters for the adapter
	 *
	 * @param enabled Whether to enable trace
	 * @param pin_protocol Configured pin protocol
	 * @param port_size Trace port width for sync mode
	 * @param trace_freq A pointer to the configured trace
	 * frequency; if it points to 0, the adapter driver must write
	 * its maximum supported rate there
	 * @param traceclkin_freq TRACECLKIN frequency provided to the TPIU in Hz
	 * @param prescaler Pointer to the SWO prescaler calculated by the
	 * adapter
	 * @returns ERROR_OK on success, an error code on failure.
	 */
	int (*config_trace)(bool enabled, enum tpiu_pin_protocol pin_protocol,
		uint32_t port_size, unsigned int *trace_freq,
		unsigned int traceclkin_freq, uint16_t *prescaler);

	/**
	 * Poll for new trace data
	 *
	 * @param buf A pointer to buffer to store received data
	 * @param size A pointer to buffer size; must be filled with
	 * the actual amount of bytes written
	 *
	 * @returns ERROR_OK on success, an error code on failure.
	 */
	int (*poll_trace)(uint8_t *buf, size_t *size);

	/** Low-level JTAG APIs */
	struct jtag_interface *jtag_ops;

	/** Low-level SWD APIs */
	const struct swd_driver *swd_ops;

	/* DAP APIs over JTAG transport */
	const struct dap_ops *dap_jtag_ops;

	/* DAP APIs over SWD transport */
	const struct dap_ops *dap_swd_ops;

	/* SWIM APIs */
	const struct swim_driver *swim_ops;
};

extern const char * const jtag_only[];

int adapter_resets(int assert_trst, int assert_srst);
int adapter_assert_reset(void);
int adapter_deassert_reset(void);
int adapter_config_trace(bool enabled, enum tpiu_pin_protocol pin_protocol,
		uint32_t port_size, unsigned int *trace_freq,
		unsigned int traceclkin_freq, uint16_t *prescaler);
int adapter_poll_trace(uint8_t *buf, size_t *size);

#endif /* OPENOCD_JTAG_INTERFACE_H */
