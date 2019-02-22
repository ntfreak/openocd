/***************************************************************************
*   Copyright (C) 2005 by Dominic Rath                                    *
*   Dominic.Rath@gmx.de                                                   *
*                                                                         *
*   Copyright (C) 2007-2010 Øyvind Harboe                                 *
*   oyvind.harboe@zylin.com                                               *
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

#ifndef OPENOCD_JTAG_JTAG_H
#define OPENOCD_JTAG_JTAG_H

#include <helper/binarybuffer.h>
#include <helper/log.h>

#ifndef DEBUG_JTAG_IOZ
#define DEBUG_JTAG_IOZ 64
#endif

/*-----</Macros>-------------------------------------------------*/

/**
 * Defines JTAG Test Access Port states.
 *
 * These definitions were gleaned from the ARM7TDMI-S Technical
 * Reference Manual and validated against several other ARM core
 * technical manuals.
 *
 * FIXME some interfaces require specific numbers be used, as they
 * are handed-off directly to their hardware implementations.
 * Fix those drivers to map as appropriate ... then pick some
 * sane set of numbers here (where 0/uninitialized == INVALID).
 */
typedef enum tap_state {
	TAP_INVALID = -1,

#if BUILD_ZY1000
	/* These are the old numbers. Leave as-is for now... */
	TAP_RESET    = 0, TAP_IDLE = 8,
	TAP_DRSELECT = 1, TAP_DRCAPTURE = 2, TAP_DRSHIFT = 3, TAP_DREXIT1 = 4,
	TAP_DRPAUSE  = 5, TAP_DREXIT2 = 6, TAP_DRUPDATE = 7,
	TAP_IRSELECT = 9, TAP_IRCAPTURE = 10, TAP_IRSHIFT = 11, TAP_IREXIT1 = 12,
	TAP_IRPAUSE  = 13, TAP_IREXIT2 = 14, TAP_IRUPDATE = 15,

#else
	/* Proper ARM recommended numbers */
	TAP_DREXIT2 = 0x0,
	TAP_DREXIT1 = 0x1,
	TAP_DRSHIFT = 0x2,
	TAP_DRPAUSE = 0x3,
	TAP_IRSELECT = 0x4,
	TAP_DRUPDATE = 0x5,
	TAP_DRCAPTURE = 0x6,
	TAP_DRSELECT = 0x7,
	TAP_IREXIT2 = 0x8,
	TAP_IREXIT1 = 0x9,
	TAP_IRSHIFT = 0xa,
	TAP_IRPAUSE = 0xb,
	TAP_IDLE = 0xc,
	TAP_IRUPDATE = 0xd,
	TAP_IRCAPTURE = 0xe,
	TAP_RESET = 0x0f,

#endif
} tap_state_t;

/**
 * Defines arguments for reset functions
 */
#define SRST_DEASSERT   0
#define SRST_ASSERT     1
#define TRST_DEASSERT   0
#define TRST_ASSERT     1

/**
 * Function tap_state_name
 * Returns a string suitable for display representing the JTAG tap_state
 */
const char *tap_state_name(tap_state_t state);

/** Provides user-friendly name lookup of TAP states. */
tap_state_t tap_state_by_name(const char *name);

/** The current TAP state of the pending JTAG command queue. */
extern tap_state_t cmd_queue_cur_state;

/**
 * This structure defines a single scan field in the scan. It provides
 * fields for the field's width and pointers to scan input and output
 * values.
 *
 * In addition, this structure includes a value and mask that is used by
 * jtag_add_dr_scan_check() to validate the value that was scanned out.
 */
struct scan_field {
	/** The number of bits this field specifies */
	int num_bits;
	/** A pointer to value to be scanned into the device */
	const uint8_t *out_value;
	/** A pointer to a 32-bit memory location for data scanned out */
	uint8_t *in_value;

	/** The value used to check the data scanned out. */
	uint8_t *check_value;
	/** The mask to go with check_value */
	uint8_t *check_mask;
};

struct jtag_tap {
	char *chip;
	char *tapname;
	char *dotted_name;
	int abs_chain_position;
	/** Is this TAP disabled after JTAG reset? */
	bool disabled_after_reset;
	/** Is this TAP currently enabled? */
	bool enabled;
	int ir_length; /**< size of instruction register */
	uint32_t ir_capture_value;
	uint8_t *expected; /**< Capture-IR expected value */
	uint32_t ir_capture_mask;
	uint8_t *expected_mask; /**< Capture-IR expected mask */
	uint32_t idcode; /**< device identification code */
	/** not all devices have idcode,
	 * we'll discover this during chain examination */
	bool hasidcode;

	/** Array of expected identification codes */
	uint32_t *expected_ids;
	/** Number of expected identification codes */
	uint8_t expected_ids_cnt;

	/** Flag saying whether to ignore version field in expected_ids[] */
	bool ignore_version;

	/** current instruction */
	uint8_t *cur_instr;
	/** Bypass register selected */
	int bypass;

	struct jtag_tap_event_action *event_action;

	/* Value to use with DP_TARGETSEL in SWD multidrop mode, otherwise
	 * DP_TARGETSEL_INVALID*/
	uint32_t multidrop_targetsel;

	struct jtag_tap *next_tap;
	/* private pointer to support none-jtag specific functions */
	void *priv;
};

void jtag_tap_init(struct jtag_tap *tap);
void jtag_tap_free(struct jtag_tap *tap);

struct jtag_tap *jtag_all_taps(void);
const char *jtag_tap_name(const struct jtag_tap *tap);
struct jtag_tap *jtag_tap_by_string(const char *dotted_name);
struct jtag_tap *jtag_tap_by_jim_obj(Jim_Interp *interp, Jim_Obj *obj);
struct jtag_tap *jtag_tap_by_position(unsigned abs_position);
struct jtag_tap *jtag_tap_next_enabled(struct jtag_tap *p);
unsigned jtag_tap_count_enabled(void);
unsigned jtag_tap_count(void);

/*
 * - TRST_ASSERTED triggers two sets of callbacks, after operations to
 *   reset the scan chain -- via TMS+TCK signaling, or deasserting the
 *   nTRST signal -- are queued:
 *
 *    + Callbacks in C code fire first, patching internal state
 *    + Then post-reset event scripts fire ... activating JTAG circuits
 *      via TCK cycles, exiting SWD mode via TMS sequences, etc
 *
 *   During those callbacks, scan chain contents have not been validated.
 *   JTAG operations that address a specific TAP (primarily DR/IR scans)
 *   must *not* be queued.
 *
 * - TAP_EVENT_SETUP is reported after TRST_ASSERTED, and after the scan
 *   chain has been validated.  JTAG operations including scans that
 *   target specific TAPs may be performed.
 *
 * - TAP_EVENT_ENABLE and TAP_EVENT_DISABLE implement TAP activation and
 *   deactivation outside the core using scripted code that understands
 *   the specific JTAG router type.  They might be triggered indirectly
 *   from EVENT_SETUP operations.
 */
enum jtag_event {
	JTAG_TRST_ASSERTED,
	JTAG_TAP_EVENT_SETUP,
	JTAG_TAP_EVENT_ENABLE,
	JTAG_TAP_EVENT_DISABLE,
};

struct jtag_tap_event_action {
	/** The event for which this action will be triggered. */
	enum jtag_event event;
	/** The interpreter to use for evaluating the @c body. */
	Jim_Interp *interp;
	/** Contains a script to 'eval' when the @c event is triggered. */
	Jim_Obj *body;
	/* next action in linked list */
	struct jtag_tap_event_action *next;
};

/**
 * Defines the function signature required for JTAG event callback
 * functions, which are added with jtag_register_event_callback()
 * and removed jtag_unregister_event_callback().
 * @param event The event to handle.
 * @param priv A pointer to data that was passed to
 *	jtag_register_event_callback().
 * @returns Must return ERROR_OK on success, or an error code on failure.
 *
 * @todo Change to return void or define a use for its return code.
 */
typedef int (*jtag_event_handler_t)(enum jtag_event event, void *priv);

int jtag_register_event_callback(jtag_event_handler_t f, void *x);
int jtag_unregister_event_callback(jtag_event_handler_t f, void *x);

int jtag_call_event_callbacks(enum jtag_event event);


/** @returns The current JTAG speed setting. */
int jtag_get_speed(int *speed);

/**
 * Given a @a speed setting, use the interface @c speed_div callback to
 * adjust the setting.
 * @param speed The speed setting to convert back to readable KHz.
 * @returns ERROR_OK if the interface has not been initialized or on success;
 *	otherwise, the error code produced by the @c speed_div callback.
 */
int jtag_get_speed_readable(int *speed);

/** Attempt to configure the interface for the specified KHz. */
int jtag_config_khz(unsigned khz);

/**
 * Attempt to enable RTCK/RCLK. If that fails, fallback to the
 * specified frequency.
 */
int jtag_config_rclk(unsigned fallback_speed_khz);

/** Retrieves the clock speed of the JTAG interface in KHz. */
unsigned jtag_get_speed_khz(void);

enum reset_types {
	RESET_NONE            = 0x0,
	RESET_HAS_TRST        = 0x1,
	RESET_HAS_SRST        = 0x2,
	RESET_TRST_AND_SRST   = 0x3,
	RESET_SRST_PULLS_TRST = 0x4,
	RESET_TRST_PULLS_SRST = 0x8,
	RESET_TRST_OPEN_DRAIN = 0x10,
	RESET_SRST_PUSH_PULL  = 0x20,
	RESET_SRST_NO_GATING  = 0x40,
	RESET_CNCT_UNDER_SRST = 0x80
};

enum reset_types jtag_get_reset_config(void);
void jtag_set_reset_config(enum reset_types type);

void jtag_set_nsrst_delay(unsigned delay);
unsigned jtag_get_nsrst_delay(void);

void jtag_set_ntrst_delay(unsigned delay);
unsigned jtag_get_ntrst_delay(void);

void jtag_set_nsrst_assert_width(unsigned delay);
unsigned jtag_get_nsrst_assert_width(void);

void jtag_set_ntrst_assert_width(unsigned delay);
unsigned jtag_get_ntrst_assert_width(void);

/** @returns The current state of TRST. */
int jtag_get_trst(void);
/** @returns The current state of SRST. */
int jtag_get_srst(void);

/** Enable or disable data scan verification checking. */
void jtag_set_verify(bool enable);
/** @returns True if data scan verification will be performed. */
bool jtag_will_verify(void);

/** Enable or disable verification of IR scan checking. */
void jtag_set_verify_capture_ir(bool enable);
/** @returns True if IR scan verification will be performed. */
bool jtag_will_verify_capture_ir(void);

/** Initialize debug adapter upon startup.  */
int adapter_init(struct command_context *cmd_ctx);

/** Shutdown the debug adapter upon program exit. */
int adapter_quit(void);

/** Set ms to sleep after jtag_execute_queue() flushes queue. Debug purposes. */
void jtag_set_flush_queue_sleep(int ms);

/**
 * Initialize JTAG chain using only a RESET reset. If init fails,
 * try reset + init.
 */
int jtag_init(struct command_context *cmd_ctx);

/** reset, then initialize JTAG chain */
int jtag_init_reset(struct command_context *cmd_ctx);
int jtag_register_commands(struct command_context *cmd_ctx);
int jtag_init_inner(struct command_context *cmd_ctx);

/**
 * @file
 * The JTAG interface can be implemented with a software or hardware fifo.
 *
 * TAP_DRSHIFT and TAP_IRSHIFT are illegal end states; however,
 * TAP_DRSHIFT/IRSHIFT can be emulated as end states, by using longer
 * scans.
 *
 * Code that is relatively insensitive to the path taken through state
 * machine (as long as it is JTAG compliant) can use @a endstate for
 * jtag_add_xxx_scan(). Otherwise, the pause state must be specified as
 * end state and a subsequent jtag_add_pathmove() must be issued.
 */

/**
 * Generate an IR SCAN with a list of scan fields with one entry for
 * each enabled TAP.
 *
 * If the input field list contains an instruction value for a TAP then
 * that is used otherwise the TAP is set to bypass.
 *
 * TAPs for which no fields are passed are marked as bypassed for
 * subsequent DR SCANs.
 *
 */
void jtag_add_ir_scan(struct jtag_tap *tap,
		struct scan_field *fields, tap_state_t endstate);
/**
 * The same as jtag_add_ir_scan except no verification is performed out
 * the output values.
 */
void jtag_add_ir_scan_noverify(struct jtag_tap *tap,
		const struct scan_field *fields, tap_state_t state);
/**
 * Scan out the bits in ir scan mode.
 *
 * If in_bits == NULL, discard incoming bits.
 */
void jtag_add_plain_ir_scan(int num_bits, const uint8_t *out_bits, uint8_t *in_bits,
		tap_state_t endstate);

/**
 * Generate a DR SCAN using the fields passed to the function.
 * For connected TAPs, the function checks in_fields and uses fields
 * specified there.  For bypassed TAPs, the function generates a dummy
 * 1-bit field.  The bypass status of TAPs is set by jtag_add_ir_scan().
 */
void jtag_add_dr_scan(struct jtag_tap *tap, int num_fields,
		const struct scan_field *fields, tap_state_t endstate);
/** A version of jtag_add_dr_scan() that uses the check_value/mask fields */
void jtag_add_dr_scan_check(struct jtag_tap *tap, int num_fields,
		struct scan_field *fields, tap_state_t endstate);
/**
 * Scan out the bits in ir scan mode.
 *
 * If in_bits == NULL, discard incoming bits.
 */
void jtag_add_plain_dr_scan(int num_bits,
		const uint8_t *out_bits, uint8_t *in_bits, tap_state_t endstate);

/**
 * Defines the type of data passed to the jtag_callback_t interface.
 * The underlying type must allow storing an @c int or pointer type.
 */
typedef intptr_t jtag_callback_data_t;

/**
 * Defines a simple JTAG callback that can allow conversions on data
 * scanned in from an interface.
 *
 * This callback should only be used for conversion that cannot fail.
 * For conversion types or checks that can fail, use the more complete
 * variant: jtag_callback_t.
 */
typedef void (*jtag_callback1_t)(jtag_callback_data_t data0);

/** A simpler version of jtag_add_callback4(). */
void jtag_add_callback(jtag_callback1_t f, jtag_callback_data_t data0);


/**
 * Defines the interface of the JTAG callback mechanism.  Such
 * callbacks can be executed once the queue has been flushed.
 *
 * The JTAG queue can be executed synchronously or asynchronously.
 * Typically for USB, the queue is executed asynchronously.  For
 * low-latency interfaces, the queue may be executed synchronously.
 *
 * The callback mechanism is very general and does not make many
 * assumptions about what the callback does or what its arguments are.
 * These callbacks are typically executed *after* the *entire* JTAG
 * queue has been executed for e.g. USB interfaces, and they are
 * guaranteed to be invoked in the order that they were queued.
 *
 * If the execution of the queue fails before the callbacks, then --
 * depending on driver implementation -- the callbacks may or may not be
 * invoked.
 *
 * @todo Make that behavior consistent.
 *
 * @param data0 Typically used to point to the data to operate on.
 * Frequently this will be the data clocked in during a shift operation.
 * @param data1 An integer big enough to use as an @c int or a pointer.
 * @param data2 An integer big enough to use as an @c int or a pointer.
 * @param data3 An integer big enough to use as an @c int or a pointer.
 * @returns an error code
 */
typedef int (*jtag_callback_t)(jtag_callback_data_t data0,
				jtag_callback_data_t data1,
				jtag_callback_data_t data2,
				jtag_callback_data_t data3);

/**
 * Run a TAP_RESET reset where the end state is TAP_RESET,
 * regardless of the start state.
 */
void jtag_add_tlr(void);

/**
 * Application code *must* assume that interfaces will
 * implement transitions between states with different
 * paths and path lengths through the state diagram. The
 * path will vary across interface and also across versions
 * of the same interface over time. Even if the OpenOCD code
 * is unchanged, the actual path taken may vary over time
 * and versions of interface firmware or PCB revisions.
 *
 * Use jtag_add_pathmove() when specific transition sequences
 * are required.
 *
 * Do not use jtag_add_pathmove() unless you need to, but do use it
 * if you have to.
 *
 * DANGER! If the target is dependent upon a particular sequence
 * of transitions for things to work correctly(e.g. as a workaround
 * for an errata that contradicts the JTAG standard), then pathmove
 * must be used, even if some jtag interfaces happen to use the
 * desired path. Worse, the jtag interface used for testing a
 * particular implementation, could happen to use the "desired"
 * path when transitioning to/from end
 * state.
 *
 * A list of unambiguous single clock state transitions, not
 * all drivers can support this, but it is required for e.g.
 * XScale and Xilinx support
 *
 * Note! TAP_RESET must not be used in the path!
 *
 * Note that the first on the list must be reachable
 * via a single transition from the current state.
 *
 * All drivers are required to implement jtag_add_pathmove().
 * However, if the pathmove sequence can not be precisely
 * executed, an interface_jtag_add_pathmove() or jtag_execute_queue()
 * must return an error. It is legal, but not recommended, that
 * a driver returns an error in all cases for a pathmove if it
 * can only implement a few transitions and therefore
 * a partial implementation of pathmove would have little practical
 * application.
 *
 * If an error occurs, jtag_error will contain one of these error codes:
 *   - ERROR_JTAG_NOT_STABLE_STATE -- The final state was not stable.
 *   - ERROR_JTAG_STATE_INVALID -- The path passed through TAP_RESET.
 *   - ERROR_JTAG_TRANSITION_INVALID -- The path includes invalid
 *     state transitions.
 */
void jtag_add_pathmove(int num_states, const tap_state_t *path);

/**
 * jtag_add_statemove() moves from the current state to @a goal_state.
 *
 * @param goal_state The final TAP state.
 * @return ERROR_OK on success, or an error code on failure.
 *
 * Moves from the current state to the goal \a state.
 * Both states must be stable.
 */
int jtag_add_statemove(tap_state_t goal_state);

/**
 * Goes to TAP_IDLE (if we're not already there), cycle
 * precisely num_cycles in the TAP_IDLE state, after which move
 * to @a endstate (unless it is also TAP_IDLE).
 *
 * @param num_cycles Number of cycles in TAP_IDLE state.  This argument
 *	may be 0, in which case this routine will navigate to @a endstate
 *	via TAP_IDLE.
 * @param endstate The final state.
 */
void jtag_add_runtest(int num_cycles, tap_state_t endstate);

/**
 * A reset of the TAP state machine can be requested.
 *
 * Whether tms or trst reset is used depends on the capabilities of
 * the target and jtag interface(reset_config  command configures this).
 *
 * srst can driver a reset of the TAP state machine and vice
 * versa
 *
 * Application code may need to examine value of jtag_reset_config
 * to determine the proper codepath
 *
 * DANGER! Even though srst drives trst, trst might not be connected to
 * the interface, and it might actually be *harmful* to assert trst in this case.
 *
 * This is why combinations such as "reset_config srst_only srst_pulls_trst"
 * are supported.
 *
 * only req_tlr_or_trst and srst can have a transition for a
 * call as the effects of transitioning both at the "same time"
 * are undefined, but when srst_pulls_trst or vice versa,
 * then trst & srst *must* be asserted together.
 */
void jtag_add_reset(int req_tlr_or_trst, int srst);

void jtag_add_sleep(uint32_t us);

int jtag_add_tms_seq(unsigned nbits, const uint8_t *seq, enum tap_state t);

/**
 * Function jtag_add_clocks
 * first checks that the state in which the clocks are to be issued is
 * stable, then queues up num_cycles clocks for transmission.
 */
void jtag_add_clocks(int num_cycles);

/**
 * For software FIFO implementations, the queued commands can be executed
 * during this call or earlier. A sw queue might decide to push out
 * some of the jtag_add_xxx() operations once the queue is "big enough".
 *
 * This fn will return an error code if any of the prior jtag_add_xxx()
 * calls caused a failure, e.g. check failure. Note that it does not
 * matter if the operation was executed *before* jtag_execute_queue(),
 * jtag_execute_queue() will still return an error code.
 *
 * All jtag_add_xxx() calls that have in_handler != NULL will have been
 * executed when this fn returns, but if what has been queued only
 * clocks data out, without reading anything back, then JTAG could
 * be running *after* jtag_execute_queue() returns. The API does
 * not define a way to flush a hw FIFO that runs *after*
 * jtag_execute_queue() returns.
 *
 * jtag_add_xxx() commands can either be executed immediately or
 * at some time between the jtag_add_xxx() fn call and jtag_execute_queue().
 */
int jtag_execute_queue(void);

/** same as jtag_execute_queue() but does not clear the error flag */
void jtag_execute_queue_noclear(void);

/** @returns the number of times the scan queue has been flushed */
int jtag_get_flush_queue_count(void);

/** Report Tcl event to all TAPs */
void jtag_notify_event(enum jtag_event);

/* can be implemented by hw + sw */
int jtag_power_dropout(int *dropout);
int jtag_srst_asserted(int *srst_asserted);

/* JTAG support functions */

/**
 * Execute jtag queue and check value with an optional mask.
 * @param field Pointer to scan field.
 * @param value Pointer to scan value.
 * @param mask Pointer to scan mask; may be NULL.
 * @returns Nothing, but calls jtag_set_error() on any error.
 */
void jtag_check_value_mask(struct scan_field *field, uint8_t *value, uint8_t *mask);

void jtag_sleep(uint32_t us);

/*
 * The JTAG subsystem defines a number of error codes,
 * using codes between -100 and -199.
 */
#define ERROR_JTAG_INIT_FAILED       (-100)
#define ERROR_JTAG_INVALID_INTERFACE (-101)
#define ERROR_JTAG_NOT_IMPLEMENTED   (-102)
#define ERROR_JTAG_TRST_ASSERTED     (-103)
#define ERROR_JTAG_QUEUE_FAILED      (-104)
#define ERROR_JTAG_NOT_STABLE_STATE  (-105)
#define ERROR_JTAG_DEVICE_ERROR      (-107)
#define ERROR_JTAG_STATE_INVALID     (-108)
#define ERROR_JTAG_TRANSITION_INVALID (-109)
#define ERROR_JTAG_INIT_SOFT_FAIL    (-110)

/**
 * Set the current JTAG core execution error, unless one was set
 * by a previous call previously.  Driver or application code must
 * use jtag_error_clear to reset jtag_error once this routine has been
 * called with a non-zero error code.
 */
void jtag_set_error(int error);
/**
 * Resets jtag_error to ERROR_OK, returning its previous value.
 * @returns The previous value of @c jtag_error.
 */
int jtag_error_clear(void);

/**
 * Return true if it's safe for a background polling task to access the
 * JTAG scan chain.  Polling may be explicitly disallowed, and is also
 * unsafe while nTRST is active or the JTAG clock is gated off.
 */
bool is_jtag_poll_safe(void);

/**
 * Return flag reporting whether JTAG polling is disallowed.
 */
bool jtag_poll_get_enabled(void);

/**
 * Assign flag reporting whether JTAG polling is disallowed.
 */
void jtag_poll_set_enabled(bool value);


/* The minidriver may have inline versions of some of the low
 * level APIs that are used in inner loops. */
#include <jtag/minidriver.h>

int jim_jtag_newtap(Jim_Interp *interp, int argc, Jim_Obj *const *argv);

#endif /* OPENOCD_JTAG_JTAG_H */
