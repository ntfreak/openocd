/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 *                                                                         *
 *   Copyright (C) 2007,2008,2009 Øyvind Harboe                            *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag.h"
#include "swd.h"
#include "interface.h"
#include <transport/transport.h>
#include <helper/jep106.h>

#ifdef HAVE_STRINGS_H
#include <strings.h>
#endif

/* SVF and XSVF are higher level JTAG command sets (for boundary scan) */
#include "svf/svf.h"
#include "xsvf/xsvf.h"

/** The number of JTAG queue flushes (for profiling and debugging purposes). */
static int jtag_flush_queue_count;

/* Sleep this # of ms after flushing the queue */
static int jtag_flush_queue_sleep;

static void jtag_add_scan_check(struct jtag_tap *active,
		void (*jtag_add_scan)(struct jtag_tap *active,
		int in_num_fields,
		const struct scan_field *in_fields,
		tap_state_t state),
		int in_num_fields, struct scan_field *in_fields, tap_state_t state);

/**
 * The jtag_error variable is set when an error occurs while executing
 * the queue.  Application code may set this using jtag_set_error(),
 * when an error occurs during processing that should be reported during
 * jtag_execute_queue().
 *
 * The value is set and cleared, but never read by normal application code.
 *
 * This value is returned (and cleared) by jtag_execute_queue().
 */
static int jtag_error = ERROR_OK;

static const char *jtag_event_strings[] = {
	[JTAG_TRST_ASSERTED] = "TAP reset",
	[JTAG_TAP_EVENT_SETUP] = "TAP setup",
	[JTAG_TAP_EVENT_ENABLE] = "TAP enabled",
	[JTAG_TAP_EVENT_DISABLE] = "TAP disabled",
};

/*
 * JTAG adapters must initialize with TRST and SRST de-asserted
 * (they're negative logic, so that means *high*).  But some
 * hardware doesn't necessarily work that way ... so set things
 * up so that jtag_init() always forces that state.
 */
static int jtag_trst = -1;
static int jtag_srst = -1;

/**
 * List all TAPs that have been created.
 */
static struct jtag_tap *__jtag_all_taps;

static enum reset_types jtag_reset_config = RESET_NONE;
tap_state_t cmd_queue_cur_state = TAP_RESET;

static bool jtag_verify_capture_ir = true;
static int jtag_verify = 1;

/* how long the OpenOCD should wait before attempting JTAG communication after reset lines
 *deasserted (in ms) */
static int adapter_nsrst_delay;	/* default to no nSRST delay */
static int jtag_ntrst_delay;/* default to no nTRST delay */
static int adapter_nsrst_assert_width;	/* width of assertion */
static int jtag_ntrst_assert_width;	/* width of assertion */

/**
 * Contains a single callback along with a pointer that will be passed
 * when an event occurs.
 */
struct jtag_event_callback {
	/** a event callback */
	jtag_event_handler_t callback;
	/** the private data to pass to the callback */
	void *priv;
	/** the next callback */
	struct jtag_event_callback *next;
};

/* callbacks to inform high-level handlers about JTAG state changes */
static struct jtag_event_callback *jtag_event_callbacks;

/* speed in kHz*/
static int speed_khz;
/* speed to fallback to when RCLK is requested but not supported */
static int rclk_fallback_speed_khz;
static enum {CLOCK_MODE_UNSELECTED, CLOCK_MODE_KHZ, CLOCK_MODE_RCLK} clock_mode;
static int jtag_speed;

/* FIXME: change name to this variable, it is not anymore JTAG only */
static struct adapter_driver *jtag;

extern struct adapter_driver *adapter_driver;

void jtag_set_flush_queue_sleep(int ms)
{
	jtag_flush_queue_sleep = ms;
}

void jtag_set_error(int error)
{
	if ((error == ERROR_OK) || (jtag_error != ERROR_OK))
		return;
	jtag_error = error;
}

int jtag_error_clear(void)
{
	int temp = jtag_error;
	jtag_error = ERROR_OK;
	return temp;
}

/************/

static bool jtag_poll = 1;

bool is_jtag_poll_safe(void)
{
	/* Polling can be disabled explicitly with set_enabled(false).
	 * It is also implicitly disabled while TRST is active and
	 * while SRST is gating the JTAG clock.
	 */
	if (!transport_is_jtag())
		return jtag_poll;

	if (!jtag_poll || jtag_trst != 0)
		return false;
	return jtag_srst == 0 || (jtag_reset_config & RESET_SRST_NO_GATING);
}

bool jtag_poll_get_enabled(void)
{
	return jtag_poll;
}

void jtag_poll_set_enabled(bool value)
{
	jtag_poll = value;
}

/************/

struct jtag_tap *jtag_all_taps(void)
{
	return __jtag_all_taps;
};

unsigned jtag_tap_count(void)
{
	struct jtag_tap *t = jtag_all_taps();
	unsigned n = 0;
	while (t) {
		n++;
		t = t->next_tap;
	}
	return n;
}

unsigned jtag_tap_count_enabled(void)
{
	struct jtag_tap *t = jtag_all_taps();
	unsigned n = 0;
	while (t) {
		if (t->enabled)
			n++;
		t = t->next_tap;
	}
	return n;
}

/** Append a new TAP to the chain of all taps. */
void jtag_tap_add(struct jtag_tap *t)
{
	unsigned jtag_num_taps = 0;

	struct jtag_tap **tap = &__jtag_all_taps;
	while (*tap != NULL) {
		jtag_num_taps++;
		tap = &(*tap)->next_tap;
	}
	*tap = t;
	t->abs_chain_position = jtag_num_taps;
}

/* returns a pointer to the n-th device in the scan chain */
struct jtag_tap *jtag_tap_by_position(unsigned n)
{
	struct jtag_tap *t = jtag_all_taps();

	while (t && n-- > 0)
		t = t->next_tap;

	return t;
}

struct jtag_tap *jtag_tap_by_string(const char *s)
{
	/* try by name first */
	struct jtag_tap *t = jtag_all_taps();

	while (t) {
		if (0 == strcmp(t->dotted_name, s))
			return t;
		t = t->next_tap;
	}

	/* no tap found by name, so try to parse the name as a number */
	unsigned n;
	if (parse_uint(s, &n) != ERROR_OK)
		return NULL;

	/* FIXME remove this numeric fallback code late June 2010, along
	 * with all info in the User's Guide that TAPs have numeric IDs.
	 * Also update "scan_chain" output to not display the numbers.
	 */
	t = jtag_tap_by_position(n);
	if (t)
		LOG_WARNING("Specify TAP '%s' by name, not number %u",
			t->dotted_name, n);

	return t;
}

struct jtag_tap *jtag_tap_next_enabled(struct jtag_tap *p)
{
	p = p ? p->next_tap : jtag_all_taps();
	while (p) {
		if (p->enabled)
			return p;
		p = p->next_tap;
	}
	return NULL;
}

const char *jtag_tap_name(const struct jtag_tap *tap)
{
	return (tap == NULL) ? "(unknown)" : tap->dotted_name;
}


int jtag_register_event_callback(jtag_event_handler_t callback, void *priv)
{
	struct jtag_event_callback **callbacks_p = &jtag_event_callbacks;

	if (callback == NULL)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (*callbacks_p) {
		while ((*callbacks_p)->next)
			callbacks_p = &((*callbacks_p)->next);
		callbacks_p = &((*callbacks_p)->next);
	}

	(*callbacks_p) = malloc(sizeof(struct jtag_event_callback));
	(*callbacks_p)->callback = callback;
	(*callbacks_p)->priv = priv;
	(*callbacks_p)->next = NULL;

	return ERROR_OK;
}

int jtag_unregister_event_callback(jtag_event_handler_t callback, void *priv)
{
	struct jtag_event_callback **p = &jtag_event_callbacks, *temp;

	if (callback == NULL)
		return ERROR_COMMAND_SYNTAX_ERROR;

	while (*p) {
		if (((*p)->priv != priv) || ((*p)->callback != callback)) {
			p = &(*p)->next;
			continue;
		}

		temp = *p;
		*p = (*p)->next;
		free(temp);
	}

	return ERROR_OK;
}

int jtag_call_event_callbacks(enum jtag_event event)
{
	struct jtag_event_callback *callback = jtag_event_callbacks;

	LOG_DEBUG("jtag event: %s", jtag_event_strings[event]);

	while (callback) {
		struct jtag_event_callback *next;

		/* callback may remove itself */
		next = callback->next;
		callback->callback(event, callback->priv);
		callback = next;
	}

	return ERROR_OK;
}

static void jtag_checks(void)
{
	assert(jtag_trst == 0);
}

static void jtag_prelude(tap_state_t state)
{
	jtag_checks();

	assert(state != TAP_INVALID);

	cmd_queue_cur_state = state;
}

void jtag_add_ir_scan_noverify(struct jtag_tap *active, const struct scan_field *in_fields,
	tap_state_t state)
{
	jtag_prelude(state);

	int retval = interface_jtag_add_ir_scan(active, in_fields, state);
	jtag_set_error(retval);
}

static void jtag_add_ir_scan_noverify_callback(struct jtag_tap *active,
	int dummy,
	const struct scan_field *in_fields,
	tap_state_t state)
{
	jtag_add_ir_scan_noverify(active, in_fields, state);
}

/* If fields->in_value is filled out, then the captured IR value will be checked */
void jtag_add_ir_scan(struct jtag_tap *active, struct scan_field *in_fields, tap_state_t state)
{
	assert(state != TAP_RESET);

	if (jtag_verify && jtag_verify_capture_ir) {
		/* 8 x 32 bit id's is enough for all invocations */

		/* if we are to run a verification of the ir scan, we need to get the input back.
		 * We may have to allocate space if the caller didn't ask for the input back.
		 */
		in_fields->check_value = active->expected;
		in_fields->check_mask = active->expected_mask;
		jtag_add_scan_check(active, jtag_add_ir_scan_noverify_callback, 1, in_fields,
			state);
	} else
		jtag_add_ir_scan_noverify(active, in_fields, state);
}

void jtag_add_plain_ir_scan(int num_bits, const uint8_t *out_bits, uint8_t *in_bits,
	tap_state_t state)
{
	assert(out_bits != NULL);
	assert(state != TAP_RESET);

	jtag_prelude(state);

	int retval = interface_jtag_add_plain_ir_scan(
			num_bits, out_bits, in_bits, state);
	jtag_set_error(retval);
}

static int jtag_check_value_inner(uint8_t *captured, uint8_t *in_check_value,
				  uint8_t *in_check_mask, int num_bits);

static int jtag_check_value_mask_callback(jtag_callback_data_t data0,
	jtag_callback_data_t data1,
	jtag_callback_data_t data2,
	jtag_callback_data_t data3)
{
	return jtag_check_value_inner((uint8_t *)data0,
		(uint8_t *)data1,
		(uint8_t *)data2,
		(int)data3);
}

static void jtag_add_scan_check(struct jtag_tap *active, void (*jtag_add_scan)(
		struct jtag_tap *active,
		int in_num_fields,
		const struct scan_field *in_fields,
		tap_state_t state),
	int in_num_fields, struct scan_field *in_fields, tap_state_t state)
{
	jtag_add_scan(active, in_num_fields, in_fields, state);

	for (int i = 0; i < in_num_fields; i++) {
		if ((in_fields[i].check_value != NULL) && (in_fields[i].in_value != NULL)) {
			/* this is synchronous for a minidriver */
			jtag_add_callback4(jtag_check_value_mask_callback,
				(jtag_callback_data_t)in_fields[i].in_value,
				(jtag_callback_data_t)in_fields[i].check_value,
				(jtag_callback_data_t)in_fields[i].check_mask,
				(jtag_callback_data_t)in_fields[i].num_bits);
		}
	}
}

void jtag_add_dr_scan_check(struct jtag_tap *active,
	int in_num_fields,
	struct scan_field *in_fields,
	tap_state_t state)
{
	if (jtag_verify)
		jtag_add_scan_check(active, jtag_add_dr_scan, in_num_fields, in_fields, state);
	else
		jtag_add_dr_scan(active, in_num_fields, in_fields, state);
}


void jtag_add_dr_scan(struct jtag_tap *active,
	int in_num_fields,
	const struct scan_field *in_fields,
	tap_state_t state)
{
	assert(state != TAP_RESET);

	jtag_prelude(state);

	int retval;
	retval = interface_jtag_add_dr_scan(active, in_num_fields, in_fields, state);
	jtag_set_error(retval);
}

void jtag_add_plain_dr_scan(int num_bits, const uint8_t *out_bits, uint8_t *in_bits,
	tap_state_t state)
{
	assert(out_bits != NULL);
	assert(state != TAP_RESET);

	jtag_prelude(state);

	int retval;
	retval = interface_jtag_add_plain_dr_scan(num_bits, out_bits, in_bits, state);
	jtag_set_error(retval);
}

void jtag_add_tlr(void)
{
	jtag_prelude(TAP_RESET);
	jtag_set_error(interface_jtag_add_tlr());

	/* NOTE: order here matches TRST path in jtag_add_reset() */
	jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
	jtag_notify_event(JTAG_TRST_ASSERTED);
}

/**
 * If supported by the underlying adapter, this clocks a raw bit sequence
 * onto TMS for switching betwen JTAG and SWD modes.
 *
 * DO NOT use this to bypass the integrity checks and logging provided
 * by the jtag_add_pathmove() and jtag_add_statemove() calls.
 *
 * @param nbits How many bits to clock out.
 * @param seq The bit sequence.  The LSB is bit 0 of seq[0].
 * @param state The JTAG tap state to record on completion.  Use
 *	TAP_INVALID to represent being in in SWD mode.
 *
 * @todo Update naming conventions to stop assuming everything is JTAG.
 */
int jtag_add_tms_seq(unsigned nbits, const uint8_t *seq, enum tap_state state)
{
	int retval;

	if (!(jtag->jtag_ops->supported & DEBUG_CAP_TMS_SEQ))
		return ERROR_JTAG_NOT_IMPLEMENTED;

	jtag_checks();
	cmd_queue_cur_state = state;

	retval = interface_add_tms_seq(nbits, seq, state);
	jtag_set_error(retval);
	return retval;
}

void jtag_add_pathmove(int num_states, const tap_state_t *path)
{
	tap_state_t cur_state = cmd_queue_cur_state;

	/* the last state has to be a stable state */
	if (!tap_is_state_stable(path[num_states - 1])) {
		LOG_ERROR("BUG: TAP path doesn't finish in a stable state");
		jtag_set_error(ERROR_JTAG_NOT_STABLE_STATE);
		return;
	}

	for (int i = 0; i < num_states; i++) {
		if (path[i] == TAP_RESET) {
			LOG_ERROR("BUG: TAP_RESET is not a valid state for pathmove sequences");
			jtag_set_error(ERROR_JTAG_STATE_INVALID);
			return;
		}

		if (tap_state_transition(cur_state, true) != path[i] &&
				tap_state_transition(cur_state, false) != path[i]) {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(cur_state), tap_state_name(path[i]));
			jtag_set_error(ERROR_JTAG_TRANSITION_INVALID);
			return;
		}
		cur_state = path[i];
	}

	jtag_checks();

	jtag_set_error(interface_jtag_add_pathmove(num_states, path));
	cmd_queue_cur_state = path[num_states - 1];
}

int jtag_add_statemove(tap_state_t goal_state)
{
	tap_state_t cur_state = cmd_queue_cur_state;

	if (goal_state != cur_state) {
		LOG_DEBUG("cur_state=%s goal_state=%s",
			tap_state_name(cur_state),
			tap_state_name(goal_state));
	}

	/* If goal is RESET, be paranoid and force that that transition
	 * (e.g. five TCK cycles, TMS high).  Else trust "cur_state".
	 */
	if (goal_state == TAP_RESET)
		jtag_add_tlr();
	else if (goal_state == cur_state)
		/* nothing to do */;

	else if (tap_is_state_stable(cur_state) && tap_is_state_stable(goal_state)) {
		unsigned tms_bits  = tap_get_tms_path(cur_state, goal_state);
		unsigned tms_count = tap_get_tms_path_len(cur_state, goal_state);
		tap_state_t moves[8];
		assert(tms_count < ARRAY_SIZE(moves));

		for (unsigned i = 0; i < tms_count; i++, tms_bits >>= 1) {
			bool bit = tms_bits & 1;

			cur_state = tap_state_transition(cur_state, bit);
			moves[i] = cur_state;
		}

		jtag_add_pathmove(tms_count, moves);
	} else if (tap_state_transition(cur_state, true)  == goal_state
			|| tap_state_transition(cur_state, false) == goal_state)
		jtag_add_pathmove(1, &goal_state);
	else
		return ERROR_FAIL;

	return ERROR_OK;
}

void jtag_add_runtest(int num_cycles, tap_state_t state)
{
	jtag_prelude(state);
	jtag_set_error(interface_jtag_add_runtest(num_cycles, state));
}


void jtag_add_clocks(int num_cycles)
{
	if (!tap_is_state_stable(cmd_queue_cur_state)) {
		LOG_ERROR("jtag_add_clocks() called with TAP in unstable state \"%s\"",
			tap_state_name(cmd_queue_cur_state));
		jtag_set_error(ERROR_JTAG_NOT_STABLE_STATE);
		return;
	}

	if (num_cycles > 0) {
		jtag_checks();
		jtag_set_error(interface_jtag_add_clocks(num_cycles));
	}
}

static int adapter_system_reset(int req_srst)
{
	int retval;

	if (req_srst) {
		if (!(jtag_reset_config & RESET_HAS_SRST)) {
			LOG_ERROR("BUG: can't assert SRST");
			return ERROR_FAIL;
		}
		req_srst = 1;
	}

	/* Maybe change SRST signal state */
	if (jtag_srst != req_srst) {
		retval = jtag->reset(0, req_srst);
		if (retval != ERROR_OK) {
			LOG_ERROR("SRST error");
			return ERROR_FAIL;
		}
		jtag_srst = req_srst;

		if (req_srst) {
			LOG_DEBUG("SRST line asserted");
			if (adapter_nsrst_assert_width)
				jtag_sleep(adapter_nsrst_assert_width * 1000);
		} else {
			LOG_DEBUG("SRST line released");
			if (adapter_nsrst_delay)
				jtag_sleep(adapter_nsrst_delay * 1000);
		}
	}

	return ERROR_OK;
}

static void legacy_jtag_add_reset(int req_tlr_or_trst, int req_srst)
{
	int trst_with_tlr = 0;
	int new_srst = 0;
	int new_trst = 0;

	/* Without SRST, we must use target-specific JTAG operations
	 * on each target; callers should not be requesting SRST when
	 * that signal doesn't exist.
	 *
	 * RESET_SRST_PULLS_TRST is a board or chip level quirk, which
	 * can kick in even if the JTAG adapter can't drive TRST.
	 */
	if (req_srst) {
		if (!(jtag_reset_config & RESET_HAS_SRST)) {
			LOG_ERROR("BUG: can't assert SRST");
			jtag_set_error(ERROR_FAIL);
			return;
		}
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) != 0
				&& !req_tlr_or_trst) {
			LOG_ERROR("BUG: can't assert only SRST");
			jtag_set_error(ERROR_FAIL);
			return;
		}
		new_srst = 1;
	}

	/* JTAG reset (entry to TAP_RESET state) can always be achieved
	 * using TCK and TMS; that may go through a TAP_{IR,DR}UPDATE
	 * state first.  TRST accelerates it, and bypasses those states.
	 *
	 * RESET_TRST_PULLS_SRST is a board or chip level quirk, which
	 * can kick in even if the JTAG adapter can't drive SRST.
	 */
	if (req_tlr_or_trst) {
		if (!(jtag_reset_config & RESET_HAS_TRST))
			trst_with_tlr = 1;
		else if ((jtag_reset_config & RESET_TRST_PULLS_SRST) != 0
			 && !req_srst)
			trst_with_tlr = 1;
		else
			new_trst = 1;
	}

	/* Maybe change TRST and/or SRST signal state */
	if (jtag_srst != new_srst || jtag_trst != new_trst) {
		int retval;

		retval = interface_jtag_add_reset(new_trst, new_srst);
		if (retval != ERROR_OK)
			jtag_set_error(retval);
		else
			retval = jtag_execute_queue();

		if (retval != ERROR_OK) {
			LOG_ERROR("TRST/SRST error");
			return;
		}
	}

	/* SRST resets everything hooked up to that signal */
	if (jtag_srst != new_srst) {
		jtag_srst = new_srst;
		if (jtag_srst) {
			LOG_DEBUG("SRST line asserted");
			if (adapter_nsrst_assert_width)
				jtag_add_sleep(adapter_nsrst_assert_width * 1000);
		} else {
			LOG_DEBUG("SRST line released");
			if (adapter_nsrst_delay)
				jtag_add_sleep(adapter_nsrst_delay * 1000);
		}
	}

	/* Maybe enter the JTAG TAP_RESET state ...
	 *  - using only TMS, TCK, and the JTAG state machine
	 *  - or else more directly, using TRST
	 *
	 * TAP_RESET should be invisible to non-debug parts of the system.
	 */
	if (trst_with_tlr) {
		LOG_DEBUG("JTAG reset with TLR instead of TRST");
		jtag_add_tlr();

	} else if (jtag_trst != new_trst) {
		jtag_trst = new_trst;
		if (jtag_trst) {
			LOG_DEBUG("TRST line asserted");
			tap_set_state(TAP_RESET);
			if (jtag_ntrst_assert_width)
				jtag_add_sleep(jtag_ntrst_assert_width * 1000);
		} else {
			LOG_DEBUG("TRST line released");
			if (jtag_ntrst_delay)
				jtag_add_sleep(jtag_ntrst_delay * 1000);

			/* We just asserted nTRST, so we're now in TAP_RESET.
			 * Inform possible listeners about this, now that
			 * JTAG instructions and data can be shifted.  This
			 * sequence must match jtag_add_tlr().
			 */
			jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
			jtag_notify_event(JTAG_TRST_ASSERTED);
		}
	}
}

/* FIXME: name is misleading; we do not plan to "add" reset into jtag queue */
void jtag_add_reset(int req_tlr_or_trst, int req_srst)
{
	int retval;
	int trst_with_tlr = 0;
	int new_srst = 0;
	int new_trst = 0;

	if (!jtag->reset) {
		legacy_jtag_add_reset(req_tlr_or_trst, req_srst);
		return;
	}

	/* Without SRST, we must use target-specific JTAG operations
	 * on each target; callers should not be requesting SRST when
	 * that signal doesn't exist.
	 *
	 * RESET_SRST_PULLS_TRST is a board or chip level quirk, which
	 * can kick in even if the JTAG adapter can't drive TRST.
	 */
	if (req_srst) {
		if (!(jtag_reset_config & RESET_HAS_SRST)) {
			LOG_ERROR("BUG: can't assert SRST");
			jtag_set_error(ERROR_FAIL);
			return;
		}
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) != 0
				&& !req_tlr_or_trst) {
			LOG_ERROR("BUG: can't assert only SRST");
			jtag_set_error(ERROR_FAIL);
			return;
		}
		new_srst = 1;
	}

	/* JTAG reset (entry to TAP_RESET state) can always be achieved
	 * using TCK and TMS; that may go through a TAP_{IR,DR}UPDATE
	 * state first.  TRST accelerates it, and bypasses those states.
	 *
	 * RESET_TRST_PULLS_SRST is a board or chip level quirk, which
	 * can kick in even if the JTAG adapter can't drive SRST.
	 */
	if (req_tlr_or_trst) {
		if (!(jtag_reset_config & RESET_HAS_TRST))
			trst_with_tlr = 1;
		else if ((jtag_reset_config & RESET_TRST_PULLS_SRST) != 0
			 && !req_srst)
			trst_with_tlr = 1;
		else
			new_trst = 1;
	}

	/* Maybe change TRST and/or SRST signal state */
	if (jtag_srst != new_srst || jtag_trst != new_trst) {
		/* guarantee jtag queue empty before changing reset status */
		jtag_execute_queue();

		retval = jtag->reset(new_trst, new_srst);
		if (retval != ERROR_OK) {
			jtag_set_error(retval);
			LOG_ERROR("TRST/SRST error");
			return;
		}
	}

	/* SRST resets everything hooked up to that signal */
	if (jtag_srst != new_srst) {
		jtag_srst = new_srst;
		if (jtag_srst) {
			LOG_DEBUG("SRST line asserted");
			if (adapter_nsrst_assert_width)
				jtag_add_sleep(adapter_nsrst_assert_width * 1000);
		} else {
			LOG_DEBUG("SRST line released");
			if (adapter_nsrst_delay)
				jtag_add_sleep(adapter_nsrst_delay * 1000);
		}
	}

	/* Maybe enter the JTAG TAP_RESET state ...
	 *  - using only TMS, TCK, and the JTAG state machine
	 *  - or else more directly, using TRST
	 *
	 * TAP_RESET should be invisible to non-debug parts of the system.
	 */
	if (trst_with_tlr) {
		LOG_DEBUG("JTAG reset with TLR instead of TRST");
		jtag_add_tlr();
		jtag_execute_queue();

	} else if (jtag_trst != new_trst) {
		jtag_trst = new_trst;
		if (jtag_trst) {
			LOG_DEBUG("TRST line asserted");
			tap_set_state(TAP_RESET);
			if (jtag_ntrst_assert_width)
				jtag_add_sleep(jtag_ntrst_assert_width * 1000);
		} else {
			LOG_DEBUG("TRST line released");
			if (jtag_ntrst_delay)
				jtag_add_sleep(jtag_ntrst_delay * 1000);

			/* We just asserted nTRST, so we're now in TAP_RESET.
			 * Inform possible listeners about this, now that
			 * JTAG instructions and data can be shifted.  This
			 * sequence must match jtag_add_tlr().
			 */
			jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
			jtag_notify_event(JTAG_TRST_ASSERTED);
		}
	}
}

void jtag_add_sleep(uint32_t us)
{
	/** @todo Here, keep_alive() appears to be a layering violation!!! */
	keep_alive();
	jtag_set_error(interface_jtag_add_sleep(us));
}

static int jtag_check_value_inner(uint8_t *captured, uint8_t *in_check_value,
	uint8_t *in_check_mask, int num_bits)
{
	int retval = ERROR_OK;
	int compare_failed;

	if (in_check_mask)
		compare_failed = buf_cmp_mask(captured, in_check_value, in_check_mask, num_bits);
	else
		compare_failed = buf_cmp(captured, in_check_value, num_bits);

	if (compare_failed) {
		char *captured_str, *in_check_value_str;
		int bits = (num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : num_bits;

		/* NOTE:  we've lost diagnostic context here -- 'which tap' */

		captured_str = buf_to_str(captured, bits, 16);
		in_check_value_str = buf_to_str(in_check_value, bits, 16);

		LOG_WARNING("Bad value '%s' captured during DR or IR scan:",
			captured_str);
		LOG_WARNING(" check_value: 0x%s", in_check_value_str);

		free(captured_str);
		free(in_check_value_str);

		if (in_check_mask) {
			char *in_check_mask_str;

			in_check_mask_str = buf_to_str(in_check_mask, bits, 16);
			LOG_WARNING(" check_mask: 0x%s", in_check_mask_str);
			free(in_check_mask_str);
		}

		retval = ERROR_JTAG_QUEUE_FAILED;
	}
	return retval;
}

void jtag_check_value_mask(struct scan_field *field, uint8_t *value, uint8_t *mask)
{
	assert(field->in_value != NULL);

	if (value == NULL) {
		/* no checking to do */
		return;
	}

	jtag_execute_queue_noclear();

	int retval = jtag_check_value_inner(field->in_value, value, mask, field->num_bits);
	jtag_set_error(retval);
}

int default_interface_jtag_execute_queue(void)
{
	if (NULL == jtag) {
		LOG_ERROR("No JTAG interface configured yet.  "
			"Issue 'init' command in startup scripts "
			"before communicating with targets.");
		return ERROR_FAIL;
	}

	if (!transport_is_jtag()) {
		/*
		 * FIXME: This should not happen!
		 * There could be old code that queues jtag commands with non jtag interfaces so, for
		 * the moment simply highlight it by log an error and return on empty execute_queue.
		 * We should fix it quitting with assert(0) because it is an internal error.
		 * The fix can be applied immediately after next release (v0.11.0 ?)
		 */
		LOG_ERROR("JTAG API jtag_execute_queue() called on non JTAG interface");
		if (!jtag->jtag_ops || !jtag->jtag_ops->execute_queue)
			return ERROR_OK;
	}

	int result = jtag->jtag_ops->execute_queue();

#if !BUILD_ZY1000
	/* Only build this if we use a regular driver with a command queue.
	 * Otherwise jtag_command_queue won't be found at compile/link time. Its
	 * definition is in jtag/commands.c, which is only built/linked by
	 * jtag/Makefile.am if MINIDRIVER_DUMMY || !MINIDRIVER, but those variables
	 * aren't accessible here. */
	struct jtag_command *cmd = jtag_command_queue;
	while (debug_level >= LOG_LVL_DEBUG && cmd) {
		switch (cmd->type) {
			case JTAG_SCAN:
				LOG_DEBUG_IO("JTAG %s SCAN to %s",
						cmd->cmd.scan->ir_scan ? "IR" : "DR",
						tap_state_name(cmd->cmd.scan->end_state));
				for (int i = 0; i < cmd->cmd.scan->num_fields; i++) {
					struct scan_field *field = cmd->cmd.scan->fields + i;
					if (field->out_value) {
						char *str = buf_to_str(field->out_value, field->num_bits, 16);
						LOG_DEBUG_IO("  %db out: %s", field->num_bits, str);
						free(str);
					}
					if (field->in_value) {
						char *str = buf_to_str(field->in_value, field->num_bits, 16);
						LOG_DEBUG_IO("  %db  in: %s", field->num_bits, str);
						free(str);
					}
				}
				break;
			case JTAG_TLR_RESET:
				LOG_DEBUG_IO("JTAG TLR RESET to %s",
						tap_state_name(cmd->cmd.statemove->end_state));
				break;
			case JTAG_RUNTEST:
				LOG_DEBUG_IO("JTAG RUNTEST %d cycles to %s",
						cmd->cmd.runtest->num_cycles,
						tap_state_name(cmd->cmd.runtest->end_state));
				break;
			case JTAG_RESET:
				{
					const char *reset_str[3] = {
						"leave", "deassert", "assert"
					};
					LOG_DEBUG_IO("JTAG RESET %s TRST, %s SRST",
							reset_str[cmd->cmd.reset->trst + 1],
							reset_str[cmd->cmd.reset->srst + 1]);
				}
				break;
			case JTAG_PATHMOVE:
				LOG_DEBUG_IO("JTAG PATHMOVE (TODO)");
				break;
			case JTAG_SLEEP:
				LOG_DEBUG_IO("JTAG SLEEP (TODO)");
				break;
			case JTAG_STABLECLOCKS:
				LOG_DEBUG_IO("JTAG STABLECLOCKS (TODO)");
				break;
			case JTAG_TMS:
				LOG_DEBUG_IO("JTAG TMS (TODO)");
				break;
			default:
				LOG_ERROR("Unknown JTAG command: %d", cmd->type);
				break;
		}
		cmd = cmd->next;
	}
#endif

	return result;
}

void jtag_execute_queue_noclear(void)
{
	jtag_flush_queue_count++;
	jtag_set_error(interface_jtag_execute_queue());

	if (jtag_flush_queue_sleep > 0) {
		/* For debug purposes it can be useful to test performance
		 * or behavior when delaying after flushing the queue,
		 * e.g. to simulate long roundtrip times.
		 */
		usleep(jtag_flush_queue_sleep * 1000);
	}
}

int jtag_get_flush_queue_count(void)
{
	return jtag_flush_queue_count;
}

int jtag_execute_queue(void)
{
	jtag_execute_queue_noclear();
	return jtag_error_clear();
}

static int jtag_reset_callback(enum jtag_event event, void *priv)
{
	struct jtag_tap *tap = priv;

	if (event == JTAG_TRST_ASSERTED) {
		tap->enabled = !tap->disabled_after_reset;

		/* current instruction is either BYPASS or IDCODE */
		buf_set_ones(tap->cur_instr, tap->ir_length);
		tap->bypass = 1;
	}

	return ERROR_OK;
}

/* sleep at least us microseconds. When we sleep more than 1000ms we
 * do an alive sleep, i.e. keep GDB alive. Note that we could starve
 * GDB if we slept for <1000ms many times.
 */
void jtag_sleep(uint32_t us)
{
	if (us < 1000)
		usleep(us);
	else
		alive_sleep((us+999)/1000);
}

#define JTAG_MAX_AUTO_TAPS 20

#define EXTRACT_JEP106_BANK(X) (((X) & 0xf00) >> 8)
#define EXTRACT_JEP106_ID(X)   (((X) & 0xfe) >> 1)
#define EXTRACT_MFG(X)  (((X) & 0xffe) >> 1)
#define EXTRACT_PART(X) (((X) & 0xffff000) >> 12)
#define EXTRACT_VER(X)  (((X) & 0xf0000000) >> 28)

/* A reserved manufacturer ID is used in END_OF_CHAIN_FLAG, so we
 * know that no valid TAP will have it as an IDCODE value.
 */
#define END_OF_CHAIN_FLAG       0xffffffff

/* a larger IR length than we ever expect to autoprobe */
#define JTAG_IRLEN_MAX          60

static int jtag_examine_chain_execute(uint8_t *idcode_buffer, unsigned num_idcode)
{
	struct scan_field field = {
		.num_bits = num_idcode * 32,
		.out_value = idcode_buffer,
		.in_value = idcode_buffer,
	};

	/* initialize to the end of chain ID value */
	for (unsigned i = 0; i < num_idcode; i++)
		buf_set_u32(idcode_buffer, i * 32, 32, END_OF_CHAIN_FLAG);

	jtag_add_plain_dr_scan(field.num_bits, field.out_value, field.in_value, TAP_DRPAUSE);
	jtag_add_tlr();
	return jtag_execute_queue();
}

static bool jtag_examine_chain_check(uint8_t *idcodes, unsigned count)
{
	uint8_t zero_check = 0x0;
	uint8_t one_check = 0xff;

	for (unsigned i = 0; i < count * 4; i++) {
		zero_check |= idcodes[i];
		one_check &= idcodes[i];
	}

	/* if there wasn't a single non-zero bit or if all bits were one,
	 * the scan is not valid.  We wrote a mix of both values; either
	 *
	 *  - There's a hardware issue (almost certainly):
	 *     + all-zeroes can mean a target stuck in JTAG reset
	 *     + all-ones tends to mean no target
	 *  - The scan chain is WAY longer than we can handle, *AND* either
	 *     + there are several hundreds of TAPs in bypass, or
	 *     + at least a few dozen TAPs all have an all-ones IDCODE
	 */
	if (zero_check == 0x00 || one_check == 0xff) {
		LOG_ERROR("JTAG scan chain interrogation failed: all %s",
			(zero_check == 0x00) ? "zeroes" : "ones");
		LOG_ERROR("Check JTAG interface, timings, target power, etc.");
		return false;
	}
	return true;
}

static void jtag_examine_chain_display(enum log_levels level, const char *msg,
	const char *name, uint32_t idcode)
{
	log_printf_lf(level, __FILE__, __LINE__, __func__,
		"JTAG tap: %s %16.16s: 0x%08x "
		"(mfg: 0x%3.3x (%s), part: 0x%4.4x, ver: 0x%1.1x)",
		name, msg,
		(unsigned int)idcode,
		(unsigned int)EXTRACT_MFG(idcode),
		jep106_manufacturer(EXTRACT_JEP106_BANK(idcode), EXTRACT_JEP106_ID(idcode)),
		(unsigned int)EXTRACT_PART(idcode),
		(unsigned int)EXTRACT_VER(idcode));
}

static bool jtag_idcode_is_final(uint32_t idcode)
{
	/*
	 * Some devices, such as AVR8, will output all 1's instead
	 * of TDI input value at end of chain. Allow those values
	 * instead of failing.
	 */
	return idcode == END_OF_CHAIN_FLAG;
}

/**
 * This helper checks that remaining bits in the examined chain data are
 * all as expected, but a single JTAG device requires only 64 bits to be
 * read back correctly.  This can help identify and diagnose problems
 * with the JTAG chain earlier, gives more helpful/explicit error messages.
 * Returns TRUE iff garbage was found.
 */
static bool jtag_examine_chain_end(uint8_t *idcodes, unsigned count, unsigned max)
{
	bool triggered = false;
	for (; count < max - 31; count += 32) {
		uint32_t idcode = buf_get_u32(idcodes, count, 32);

		/* do not trigger the warning if the data looks good */
		if (jtag_idcode_is_final(idcode))
			continue;
		LOG_WARNING("Unexpected idcode after end of chain: %d 0x%08x",
			count, (unsigned int)idcode);
		triggered = true;
	}
	return triggered;
}

static bool jtag_examine_chain_match_tap(const struct jtag_tap *tap)
{

	if (tap->expected_ids_cnt == 0 || !tap->hasidcode)
		return true;

	/* optionally ignore the JTAG version field - bits 28-31 of IDCODE */
	uint32_t mask = tap->ignore_version ? ~(0xfU << 28) : ~0U;
	uint32_t idcode = tap->idcode & mask;

	/* Loop over the expected identification codes and test for a match */
	for (unsigned ii = 0; ii < tap->expected_ids_cnt; ii++) {
		uint32_t expected = tap->expected_ids[ii] & mask;

		if (idcode == expected)
			return true;

		/* treat "-expected-id 0" as a "don't-warn" wildcard */
		if (0 == tap->expected_ids[ii])
			return true;
	}

	/* If none of the expected ids matched, warn */
	jtag_examine_chain_display(LOG_LVL_WARNING, "UNEXPECTED",
		tap->dotted_name, tap->idcode);
	for (unsigned ii = 0; ii < tap->expected_ids_cnt; ii++) {
		char msg[32];

		snprintf(msg, sizeof(msg), "expected %u of %u", ii + 1, tap->expected_ids_cnt);
		jtag_examine_chain_display(LOG_LVL_ERROR, msg,
			tap->dotted_name, tap->expected_ids[ii]);
	}
	return false;
}

/* Try to examine chain layout according to IEEE 1149.1 §12
 * This is called a "blind interrogation" of the scan chain.
 */
static int jtag_examine_chain(void)
{
	int retval;
	unsigned max_taps = jtag_tap_count();

	/* Autoprobe up to this many. */
	if (max_taps < JTAG_MAX_AUTO_TAPS)
		max_taps = JTAG_MAX_AUTO_TAPS;

	/* Add room for end-of-chain marker. */
	max_taps++;

	uint8_t *idcode_buffer = calloc(4, max_taps);
	if (idcode_buffer == NULL)
		return ERROR_JTAG_INIT_FAILED;

	/* DR scan to collect BYPASS or IDCODE register contents.
	 * Then make sure the scan data has both ones and zeroes.
	 */
	LOG_DEBUG("DR scan interrogation for IDCODE/BYPASS");
	retval = jtag_examine_chain_execute(idcode_buffer, max_taps);
	if (retval != ERROR_OK)
		goto out;
	if (!jtag_examine_chain_check(idcode_buffer, max_taps)) {
		retval = ERROR_JTAG_INIT_FAILED;
		goto out;
	}

	/* Point at the 1st predefined tap, if any */
	struct jtag_tap *tap = jtag_tap_next_enabled(NULL);

	unsigned bit_count = 0;
	unsigned autocount = 0;
	for (unsigned i = 0; i < max_taps; i++) {
		assert(bit_count < max_taps * 32);
		uint32_t idcode = buf_get_u32(idcode_buffer, bit_count, 32);

		/* No predefined TAP? Auto-probe. */
		if (tap == NULL) {
			/* Is there another TAP? */
			if (jtag_idcode_is_final(idcode))
				break;

			/* Default everything in this TAP except IR length.
			 *
			 * REVISIT create a jtag_alloc(chip, tap) routine, and
			 * share it with jim_newtap_cmd().
			 */
			tap = calloc(1, sizeof *tap);
			if (!tap) {
				retval = ERROR_FAIL;
				goto out;
			}

			tap->chip = alloc_printf("auto%u", autocount++);
			tap->tapname = strdup("tap");
			tap->dotted_name = alloc_printf("%s.%s", tap->chip, tap->tapname);

			tap->ir_length = 0; /* ... signifying irlen autoprobe */
			tap->ir_capture_mask = 0x03;
			tap->ir_capture_value = 0x01;

			tap->enabled = true;

			jtag_tap_init(tap);
		}

		if ((idcode & 1) == 0) {
			/* Zero for LSB indicates a device in bypass */
			LOG_INFO("TAP %s does not have valid IDCODE (idcode=0x%x)",
					tap->dotted_name, idcode);
			tap->hasidcode = false;
			tap->idcode = 0;

			bit_count += 1;
		} else {
			/* Friendly devices support IDCODE */
			tap->hasidcode = true;
			tap->idcode = idcode;
			jtag_examine_chain_display(LOG_LVL_INFO, "tap/device found", tap->dotted_name, idcode);

			bit_count += 32;
		}

		/* ensure the TAP ID matches what was expected */
		if (!jtag_examine_chain_match_tap(tap))
			retval = ERROR_JTAG_INIT_SOFT_FAIL;

		tap = jtag_tap_next_enabled(tap);
	}

	/* After those IDCODE or BYPASS register values should be
	 * only the data we fed into the scan chain.
	 */
	if (jtag_examine_chain_end(idcode_buffer, bit_count, max_taps * 32)) {
		LOG_ERROR("double-check your JTAG setup (interface, speed, ...)");
		retval = ERROR_JTAG_INIT_FAILED;
		goto out;
	}

	/* Return success or, for backwards compatibility if only
	 * some IDCODE values mismatched, a soft/continuable fault.
	 */
out:
	free(idcode_buffer);
	return retval;
}

/*
 * Validate the date loaded by entry to the Capture-IR state, to help
 * find errors related to scan chain configuration (wrong IR lengths)
 * or communication.
 *
 * Entry state can be anything.  On non-error exit, all TAPs are in
 * bypass mode.  On error exits, the scan chain is reset.
 */
static int jtag_validate_ircapture(void)
{
	struct jtag_tap *tap;
	int total_ir_length = 0;
	uint8_t *ir_test = NULL;
	struct scan_field field;
	uint64_t val;
	int chain_pos = 0;
	int retval;

	/* when autoprobing, accomodate huge IR lengths */
	for (tap = NULL, total_ir_length = 0;
			(tap = jtag_tap_next_enabled(tap)) != NULL;
			total_ir_length += tap->ir_length) {
		if (tap->ir_length == 0)
			total_ir_length += JTAG_IRLEN_MAX;
	}

	/* increase length to add 2 bit sentinel after scan */
	total_ir_length += 2;

	ir_test = malloc(DIV_ROUND_UP(total_ir_length, 8));
	if (ir_test == NULL)
		return ERROR_FAIL;

	/* after this scan, all TAPs will capture BYPASS instructions */
	buf_set_ones(ir_test, total_ir_length);

	field.num_bits = total_ir_length;
	field.out_value = ir_test;
	field.in_value = ir_test;

	jtag_add_plain_ir_scan(field.num_bits, field.out_value, field.in_value, TAP_IDLE);

	LOG_DEBUG("IR capture validation scan");
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		goto done;

	tap = NULL;
	chain_pos = 0;

	for (;; ) {
		tap = jtag_tap_next_enabled(tap);
		if (tap == NULL)
			break;

		/* If we're autoprobing, guess IR lengths.  They must be at
		 * least two bits.  Guessing will fail if (a) any TAP does
		 * not conform to the JTAG spec; or (b) when the upper bits
		 * captured from some conforming TAP are nonzero.  Or if
		 * (c) an IR length is longer than JTAG_IRLEN_MAX bits,
		 * an implementation limit, which could someday be raised.
		 *
		 * REVISIT optimization:  if there's a *single* TAP we can
		 * lift restrictions (a) and (b) by scanning a recognizable
		 * pattern before the all-ones BYPASS.  Check for where the
		 * pattern starts in the result, instead of an 0...01 value.
		 *
		 * REVISIT alternative approach: escape to some tcl code
		 * which could provide more knowledge, based on IDCODE; and
		 * only guess when that has no success.
		 */
		if (tap->ir_length == 0) {
			tap->ir_length = 2;
			while ((val = buf_get_u64(ir_test, chain_pos, tap->ir_length + 1)) == 1
					&& tap->ir_length < JTAG_IRLEN_MAX) {
				tap->ir_length++;
			}
			LOG_WARNING("AUTO %s - use \"jtag newtap " "%s %s -irlen %d "
					"-expected-id 0x%08" PRIx32 "\"",
					tap->dotted_name, tap->chip, tap->tapname, tap->ir_length, tap->idcode);
		}

		/* Validate the two LSBs, which must be 01 per JTAG spec.
		 *
		 * Or ... more bits could be provided by TAP declaration.
		 * Plus, some taps (notably in i.MX series chips) violate
		 * this part of the JTAG spec, so their capture mask/value
		 * attributes might disable this test.
		 */
		val = buf_get_u64(ir_test, chain_pos, tap->ir_length);
		if ((val & tap->ir_capture_mask) != tap->ir_capture_value) {
			LOG_ERROR("%s: IR capture error; saw 0x%0*" PRIx64 " not 0x%0*" PRIx32,
				jtag_tap_name(tap),
				(tap->ir_length + 7) / tap->ir_length, val,
				(tap->ir_length + 7) / tap->ir_length, tap->ir_capture_value);

			retval = ERROR_JTAG_INIT_FAILED;
			goto done;
		}
		LOG_DEBUG("%s: IR capture 0x%0*" PRIx64, jtag_tap_name(tap),
			(tap->ir_length + 7) / tap->ir_length, val);
		chain_pos += tap->ir_length;
	}

	/* verify the '11' sentinel we wrote is returned at the end */
	val = buf_get_u64(ir_test, chain_pos, 2);
	if (val != 0x3) {
		char *cbuf = buf_to_str(ir_test, total_ir_length, 16);

		LOG_ERROR("IR capture error at bit %d, saw 0x%s not 0x...3",
			chain_pos, cbuf);
		free(cbuf);
		retval = ERROR_JTAG_INIT_FAILED;
	}

done:
	free(ir_test);
	if (retval != ERROR_OK) {
		jtag_add_tlr();
		jtag_execute_queue();
	}
	return retval;
}

void jtag_tap_init(struct jtag_tap *tap)
{
	unsigned ir_len_bits;
	unsigned ir_len_bytes;

	/* if we're autoprobing, cope with potentially huge ir_length */
	ir_len_bits = tap->ir_length ? : JTAG_IRLEN_MAX;
	ir_len_bytes = DIV_ROUND_UP(ir_len_bits, 8);

	tap->expected = calloc(1, ir_len_bytes);
	tap->expected_mask = calloc(1, ir_len_bytes);
	tap->cur_instr = malloc(ir_len_bytes);

	/** @todo cope better with ir_length bigger than 32 bits */
	if (ir_len_bits > 32)
		ir_len_bits = 32;

	buf_set_u32(tap->expected, 0, ir_len_bits, tap->ir_capture_value);
	buf_set_u32(tap->expected_mask, 0, ir_len_bits, tap->ir_capture_mask);

	/* TAP will be in bypass mode after jtag_validate_ircapture() */
	tap->bypass = 1;
	buf_set_ones(tap->cur_instr, tap->ir_length);

	/* register the reset callback for the TAP */
	jtag_register_event_callback(&jtag_reset_callback, tap);
	jtag_tap_add(tap);

	LOG_DEBUG("Created Tap: %s @ abs position %d, "
			"irlen %d, capture: 0x%x mask: 0x%x", tap->dotted_name,
			tap->abs_chain_position, tap->ir_length,
			(unsigned) tap->ir_capture_value,
			(unsigned) tap->ir_capture_mask);
}

void jtag_tap_free(struct jtag_tap *tap)
{
	jtag_unregister_event_callback(&jtag_reset_callback, tap);

	struct jtag_tap_event_action *jteap = tap->event_action;
	while (jteap) {
		struct jtag_tap_event_action *next = jteap->next;
		Jim_DecrRefCount(jteap->interp, jteap->body);
		free(jteap);
		jteap = next;
	}

	free(tap->expected);
	free(tap->expected_mask);
	free(tap->expected_ids);
	free(tap->cur_instr);
	free(tap->chip);
	free(tap->tapname);
	free(tap->dotted_name);
	free(tap);
}

/**
 * Do low-level setup like initializing registers, output signals,
 * and clocking.
 */
int adapter_init(struct command_context *cmd_ctx)
{
	if (jtag)
		return ERROR_OK;

	if (!adapter_driver) {
		/* nothing was previously specified by "adapter driver" command */
		LOG_ERROR("Debug Adapter has to be specified, "
			"see \"adapter driver\" command");
		return ERROR_JTAG_INVALID_INTERFACE;
	}

	int retval;
	retval = adapter_driver->init();
	if (retval != ERROR_OK)
		return retval;
	jtag = adapter_driver;

	if (jtag->speed == NULL) {
		LOG_INFO("This adapter doesn't support configurable speed");
		return ERROR_OK;
	}

	if (CLOCK_MODE_UNSELECTED == clock_mode) {
		LOG_ERROR("An adapter speed is not selected in the init script."
			" Insert a call to \"adapter speed\" or \"jtag_rclk\" to proceed.");
		return ERROR_JTAG_INIT_FAILED;
	}

	int requested_khz = jtag_get_speed_khz();
	int actual_khz = requested_khz;
	int jtag_speed_var = 0;
	retval = jtag_get_speed(&jtag_speed_var);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag->speed(jtag_speed_var);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_get_speed_readable(&actual_khz);
	if (ERROR_OK != retval)
		LOG_INFO("adapter-specific clock speed value %d", jtag_speed_var);
	else if (actual_khz) {
		/* Adaptive clocking -- JTAG-specific */
		if ((CLOCK_MODE_RCLK == clock_mode)
				|| ((CLOCK_MODE_KHZ == clock_mode) && !requested_khz)) {
			LOG_INFO("RCLK (adaptive clock speed) not supported - fallback to %d kHz"
			, actual_khz);
		} else
			LOG_INFO("clock speed %d kHz", actual_khz);
	} else
		LOG_INFO("RCLK (adaptive clock speed)");

	return ERROR_OK;
}

int jtag_init_inner(struct command_context *cmd_ctx)
{
	struct jtag_tap *tap;
	int retval;
	bool issue_setup = true;

	LOG_DEBUG("Init JTAG chain");

	tap = jtag_tap_next_enabled(NULL);
	if (tap == NULL) {
		/* Once JTAG itself is properly set up, and the scan chain
		 * isn't absurdly large, IDCODE autoprobe should work fine.
		 *
		 * But ... IRLEN autoprobe can fail even on systems which
		 * are fully conformant to JTAG.  Also, JTAG setup can be
		 * quite finicky on some systems.
		 *
		 * REVISIT: if TAP autoprobe works OK, then in many cases
		 * we could escape to tcl code and set up targets based on
		 * the TAP's IDCODE values.
		 */
		LOG_WARNING("There are no enabled taps.  "
			"AUTO PROBING MIGHT NOT WORK!!");

		/* REVISIT default clock will often be too fast ... */
	}

	jtag_add_tlr();
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	/* Examine DR values first.  This discovers problems which will
	 * prevent communication ... hardware issues like TDO stuck, or
	 * configuring the wrong number of (enabled) TAPs.
	 */
	retval = jtag_examine_chain();
	switch (retval) {
		case ERROR_OK:
			/* complete success */
			break;
		default:
			/* For backward compatibility reasons, try coping with
			 * configuration errors involving only ID mismatches.
			 * We might be able to talk to the devices.
			 *
			 * Also the device might be powered down during startup.
			 *
			 * After OpenOCD starts, we can try to power on the device
			 * and run a reset.
			 */
			LOG_ERROR("Trying to use configured scan chain anyway...");
			issue_setup = false;
			break;
	}

	/* Now look at IR values.  Problems here will prevent real
	 * communication.  They mostly mean that the IR length is
	 * wrong ... or that the IR capture value is wrong.  (The
	 * latter is uncommon, but easily worked around:  provide
	 * ircapture/irmask values during TAP setup.)
	 */
	retval = jtag_validate_ircapture();
	if (retval != ERROR_OK) {
		/* The target might be powered down. The user
		 * can power it up and reset it after firing
		 * up OpenOCD.
		 */
		issue_setup = false;
	}

	if (issue_setup)
		jtag_notify_event(JTAG_TAP_EVENT_SETUP);
	else
		LOG_WARNING("Bypassing JTAG setup events due to errors");


	return ERROR_OK;
}

int adapter_quit(void)
{
	if (jtag && jtag->quit) {
		/* close the JTAG interface */
		int result = jtag->quit();
		if (ERROR_OK != result)
			LOG_ERROR("failed: %d", result);
	}

	struct jtag_tap *t = jtag_all_taps();
	while (t) {
		struct jtag_tap *n = t->next_tap;
		jtag_tap_free(t);
		t = n;
	}

	return ERROR_OK;
}

int swd_init_reset(struct command_context *cmd_ctx)
{
	int retval, retval1;

	retval = adapter_init(cmd_ctx);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("Initializing with hard SRST reset");

	if (jtag_reset_config & RESET_HAS_SRST)
		retval = adapter_system_reset(1);
	retval1 = adapter_system_reset(0);

	return (retval == ERROR_OK) ? retval1 : retval;
}

int jtag_init_reset(struct command_context *cmd_ctx)
{
	int retval = adapter_init(cmd_ctx);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("Initializing with hard TRST+SRST reset");

	/*
	 * This procedure is used by default when OpenOCD triggers a reset.
	 * It's now done through an overridable Tcl "init_reset" wrapper.
	 *
	 * This started out as a more powerful "get JTAG working" reset than
	 * jtag_init_inner(), applying TRST because some chips won't activate
	 * JTAG without a TRST cycle (presumed to be async, though some of
	 * those chips synchronize JTAG activation using TCK).
	 *
	 * But some chips only activate JTAG as part of an SRST cycle; SRST
	 * got mixed in.  So it became a hard reset routine, which got used
	 * in more places, and which coped with JTAG reset being forced as
	 * part of SRST (srst_pulls_trst).
	 *
	 * And even more corner cases started to surface:  TRST and/or SRST
	 * assertion timings matter; some chips need other JTAG operations;
	 * TRST/SRST sequences can need to be different from these, etc.
	 *
	 * Systems should override that wrapper to support system-specific
	 * requirements that this not-fully-generic code doesn't handle.
	 *
	 * REVISIT once Tcl code can read the reset_config modes, this won't
	 * need to be a C routine at all...
	 */
	if (jtag_reset_config & RESET_HAS_SRST) {
		jtag_add_reset(1, 1);
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) == 0)
			jtag_add_reset(0, 1);
	} else {
		jtag_add_reset(1, 0);	/* TAP_RESET, using TMS+TCK or TRST */
	}

	/* some targets enable us to connect with srst asserted */
	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING)
			jtag_add_reset(0, 1);
		else {
			LOG_WARNING("\'srst_nogate\' reset_config option is required");
			jtag_add_reset(0, 0);
		}
	} else
		jtag_add_reset(0, 0);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	/* Check that we can communication on the JTAG chain + eventually we want to
	 * be able to perform enumeration only after OpenOCD has started
	 * telnet and GDB server
	 *
	 * That would allow users to more easily perform any magic they need to before
	 * reset happens.
	 */
	return jtag_init_inner(cmd_ctx);
}

int jtag_init(struct command_context *cmd_ctx)
{
	int retval = adapter_init(cmd_ctx);
	if (retval != ERROR_OK)
		return retval;

	/* guard against oddball hardware: force resets to be inactive */
	jtag_add_reset(0, 0);

	/* some targets enable us to connect with srst asserted */
	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING)
			jtag_add_reset(0, 1);
		else
			LOG_WARNING("\'srst_nogate\' reset_config option is required");
	}
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (Jim_Eval_Named(cmd_ctx->interp, "jtag_init", __FILE__, __LINE__) != JIM_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

unsigned jtag_get_speed_khz(void)
{
	return speed_khz;
}

static int adapter_khz_to_speed(unsigned khz, int *speed)
{
	LOG_DEBUG("convert khz to interface specific speed value");
	speed_khz = khz;
	if (!jtag)
		return ERROR_OK;
	LOG_DEBUG("have interface set up");
	if (!jtag->khz) {
		LOG_ERROR("Translation from khz to jtag_speed not implemented");
		return ERROR_FAIL;
	}
	int speed_div1;
	int retval = jtag->khz(jtag_get_speed_khz(), &speed_div1);
	if (ERROR_OK != retval)
		return retval;
	*speed = speed_div1;
	return ERROR_OK;
}

static int jtag_rclk_to_speed(unsigned fallback_speed_khz, int *speed)
{
	int retval = adapter_khz_to_speed(0, speed);
	if ((ERROR_OK != retval) && fallback_speed_khz) {
		LOG_DEBUG("trying fallback speed...");
		retval = adapter_khz_to_speed(fallback_speed_khz, speed);
	}
	return retval;
}

static int jtag_set_speed(int speed)
{
	jtag_speed = speed;
	/* this command can be called during CONFIG,
	 * in which case jtag isn't initialized */
	return jtag ? jtag->speed(speed) : ERROR_OK;
}

int jtag_config_khz(unsigned khz)
{
	LOG_DEBUG("handle jtag khz");
	clock_mode = CLOCK_MODE_KHZ;
	int speed = 0;
	int retval = adapter_khz_to_speed(khz, &speed);
	return (ERROR_OK != retval) ? retval : jtag_set_speed(speed);
}

int jtag_config_rclk(unsigned fallback_speed_khz)
{
	LOG_DEBUG("handle jtag rclk");
	clock_mode = CLOCK_MODE_RCLK;
	rclk_fallback_speed_khz = fallback_speed_khz;
	int speed = 0;
	int retval = jtag_rclk_to_speed(fallback_speed_khz, &speed);
	return (ERROR_OK != retval) ? retval : jtag_set_speed(speed);
}

int jtag_get_speed(int *speed)
{
	switch (clock_mode) {
		case CLOCK_MODE_KHZ:
			adapter_khz_to_speed(jtag_get_speed_khz(), speed);
			break;
		case CLOCK_MODE_RCLK:
			jtag_rclk_to_speed(rclk_fallback_speed_khz, speed);
			break;
		default:
			LOG_ERROR("BUG: unknown jtag clock mode");
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

int jtag_get_speed_readable(int *khz)
{
	int jtag_speed_var = 0;
	int retval = jtag_get_speed(&jtag_speed_var);
	if (retval != ERROR_OK)
		return retval;
	if (!jtag)
		return ERROR_OK;
	if (!jtag->speed_div) {
		LOG_ERROR("Translation from jtag_speed to khz not implemented");
		return ERROR_FAIL;
	}
	return jtag->speed_div(jtag_speed_var, khz);
}

void jtag_set_verify(bool enable)
{
	jtag_verify = enable;
}

bool jtag_will_verify()
{
	return jtag_verify;
}

void jtag_set_verify_capture_ir(bool enable)
{
	jtag_verify_capture_ir = enable;
}

bool jtag_will_verify_capture_ir()
{
	return jtag_verify_capture_ir;
}

int jtag_power_dropout(int *dropout)
{
	if (jtag == NULL) {
		/* TODO: as the jtag interface is not valid all
		 * we can do at the moment is exit OpenOCD */
		LOG_ERROR("No Valid JTAG Interface Configured.");
		exit(-1);
	}
	if (jtag->power_dropout)
		return jtag->power_dropout(dropout);

	*dropout = 0; /* by default we can't detect power dropout */
	return ERROR_OK;
}

int jtag_srst_asserted(int *srst_asserted)
{
	if (jtag->srst_asserted)
		return jtag->srst_asserted(srst_asserted);

	*srst_asserted = 0; /* by default we can't detect srst asserted */
	return ERROR_OK;
}

enum reset_types jtag_get_reset_config(void)
{
	return jtag_reset_config;
}
void jtag_set_reset_config(enum reset_types type)
{
	jtag_reset_config = type;
}

int jtag_get_trst(void)
{
	return jtag_trst == 1;
}
int jtag_get_srst(void)
{
	return jtag_srst == 1;
}

void jtag_set_nsrst_delay(unsigned delay)
{
	adapter_nsrst_delay = delay;
}
unsigned jtag_get_nsrst_delay(void)
{
	return adapter_nsrst_delay;
}
void jtag_set_ntrst_delay(unsigned delay)
{
	jtag_ntrst_delay = delay;
}
unsigned jtag_get_ntrst_delay(void)
{
	return jtag_ntrst_delay;
}


void jtag_set_nsrst_assert_width(unsigned delay)
{
	adapter_nsrst_assert_width = delay;
}
unsigned jtag_get_nsrst_assert_width(void)
{
	return adapter_nsrst_assert_width;
}
void jtag_set_ntrst_assert_width(unsigned delay)
{
	jtag_ntrst_assert_width = delay;
}
unsigned jtag_get_ntrst_assert_width(void)
{
	return jtag_ntrst_assert_width;
}

static int jtag_select(struct command_context *ctx)
{
	int retval;

	/* NOTE:  interface init must already have been done.
	 * That works with only C code ... no Tcl glue required.
	 */

	retval = jtag_register_commands(ctx);

	if (retval != ERROR_OK)
		return retval;

	retval = svf_register_commands(ctx);

	if (retval != ERROR_OK)
		return retval;

	return xsvf_register_commands(ctx);
}

static struct transport jtag_transport = {
	.name = "jtag",
	.select = jtag_select,
	.init = jtag_init,
};

static void jtag_constructor(void) __attribute__((constructor));
static void jtag_constructor(void)
{
	transport_register(&jtag_transport);
}

/** Returns true if the current debug session
 * is using JTAG as its transport.
 */
bool transport_is_jtag(void)
{
	return get_current_transport() == &jtag_transport;
}

int adapter_resets(int trst, int srst)
{
	if (get_current_transport() == NULL) {
		LOG_ERROR("transport is not selected");
		return ERROR_FAIL;
	}

	if (transport_is_jtag()) {
		if (srst == SRST_ASSERT && !(jtag_reset_config & RESET_HAS_SRST)) {
			LOG_ERROR("adapter has no srst signal");
			return ERROR_FAIL;
		}

		/* adapters without trst signal will eventually use tlr sequence */
		jtag_add_reset(trst, srst);
		return ERROR_OK;
	} else if (transport_is_swd() || transport_is_hla()) {
		if (trst == TRST_ASSERT) {
			LOG_ERROR("transport %s has no trst signal",
				get_current_transport()->name);
			return ERROR_FAIL;
		}

		if (srst == SRST_ASSERT && !(jtag_reset_config & RESET_HAS_SRST)) {
			LOG_ERROR("adapter has no srst signal");
			return ERROR_FAIL;
		}
		adapter_system_reset(srst);
		return ERROR_OK;
	}

	if (trst == TRST_DEASSERT && srst == SRST_DEASSERT)
		return ERROR_OK;

	LOG_ERROR("reset is not supported on transport %s",
		get_current_transport()->name);

	return ERROR_FAIL;
}

int adapter_assert_reset(void)
{
	if (transport_is_jtag()) {
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
			jtag_add_reset(1, 1);
		else
			jtag_add_reset(0, 1);
		return ERROR_OK;
	} else if (transport_is_swd() || transport_is_hla() ||
			   transport_is_dapdirect_jtag() || transport_is_dapdirect_swd())
		return adapter_system_reset(1);
	else if (get_current_transport() != NULL)
		LOG_ERROR("reset is not supported on %s",
			get_current_transport()->name);
	else
		LOG_ERROR("transport is not selected");
	return ERROR_FAIL;
}

int adapter_deassert_reset(void)
{
	if (transport_is_jtag()) {
		jtag_add_reset(0, 0);
		return ERROR_OK;
	} else if (transport_is_swd() || transport_is_hla() ||
			 transport_is_dapdirect_jtag() || transport_is_dapdirect_swd())
		return adapter_system_reset(0);
	else if (get_current_transport() != NULL)
		LOG_ERROR("reset is not supported on %s",
			get_current_transport()->name);
	else
		LOG_ERROR("transport is not selected");
	return ERROR_FAIL;
}

int adapter_config_trace(bool enabled, enum tpiu_pin_protocol pin_protocol,
		uint32_t port_size, unsigned int *trace_freq,
		unsigned int traceclkin_freq, uint16_t *prescaler)
{
	if (jtag->config_trace) {
		return jtag->config_trace(enabled, pin_protocol, port_size, trace_freq,
			traceclkin_freq, prescaler);
	} else if (enabled) {
		LOG_ERROR("The selected interface does not support tracing");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int adapter_poll_trace(uint8_t *buf, size_t *size)
{
	if (jtag->poll_trace)
		return jtag->poll_trace(buf, size);

	return ERROR_FAIL;
}
