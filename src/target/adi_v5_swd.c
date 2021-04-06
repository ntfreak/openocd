/***************************************************************************
 *
 *   Copyright (C) 2010 by David Brownell
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
 * @file
 * Utilities to support ARM "Serial Wire Debug" (SWD), a low pin-count debug
 * link protocol used in cases where JTAG is not wanted.  This is coupled to
 * recent versions of ARM's "CoreSight" debug framework.  This specific code
 * is a transport level interface, with "target/arm_adi_v5.[hc]" code
 * understanding operation semantics, shared with the JTAG transport.
 *
 * Single-DAP support only.
 *
 * for details, see "ARM IHI 0031A"
 * ARM Debug Interface v5 Architecture Specification
 * especially section 5.3 for SWD protocol
 *
 * On many chips (most current Cortex-M3 parts) SWD is a run-time alternative
 * to JTAG.  Boards may support one or both.  There are also SWD-only chips,
 * (using SW-DP not SWJ-DP).
 *
 * Even boards that also support JTAG can benefit from SWD support, because
 * usually there's no way to access the SWO trace view mechanism in JTAG mode.
 * That is, trace access may require SWD support.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>

#include <transport/transport.h>
#include <jtag/interface.h>

#include <jtag/swd.h>

static bool do_sync;

static struct adiv5_dap *swd_multidrop_selected_dap;


static int swd_queue_dp_write_inner(struct adiv5_dap *dap, unsigned int reg,
		uint32_t data);


static bool swd_dap_is_multidrop(struct adiv5_dap *dap)
{
	return dap->tap->multidrop_targetsel != DP_TARGETSEL_INVALID;
}

static int swd_send_sequence(struct adiv5_dap *dap, enum swd_special_seq seq)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	return swd->switch_seq(seq);
}

static void swd_finish_read(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	if (dap->last_read != NULL) {
		swd->read_reg(swd_cmd(true, false, DP_RDBUFF), dap->last_read, 0);
		dap->last_read = NULL;
	}
}

static void swd_clear_sticky_errors(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	swd->write_reg(swd_cmd(false, false, DP_ABORT),
		STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
}

static int swd_run_inner(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	int retval;

	retval = swd->run();

	if (retval != ERROR_OK) {
		/* fault response */
		dap->do_reconnect = true;
	}

	return retval;
}

static inline int check_sync(struct adiv5_dap *dap)
{
	return do_sync ? swd_run_inner(dap) : ERROR_OK;
}

/** Select the DP register bank matching bits 7:4 of reg. */
static int swd_queue_dp_bankselect(struct adiv5_dap *dap, unsigned int reg)
{
	/* Only register address 4 is banked. */
	if ((reg & 0xf) != 4)
		return ERROR_OK;

	uint32_t select_dp_bank = (reg & 0x000000F0) >> 4;
	uint32_t sel = select_dp_bank
			| (dap->select & (DP_SELECT_APSEL | DP_SELECT_APBANK));

	if (sel == dap->select)
		return ERROR_OK;

	dap->select = sel;

	int retval = swd_queue_dp_write_inner(dap, DP_SELECT, sel);
	if (retval != ERROR_OK)
		dap->select = DP_SELECT_INVALID;

	return retval;
}

static int swd_queue_dp_read_inner(struct adiv5_dap *dap, unsigned int reg,
		uint32_t *data)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	int retval = swd_queue_dp_bankselect(dap, reg);
	if (retval != ERROR_OK)
		return retval;

	swd->read_reg(swd_cmd(true, false, reg), data, 0);

	return check_sync(dap);
}

static int swd_queue_dp_write_inner(struct adiv5_dap *dap, unsigned int reg,
		uint32_t data)
{
	int retval;
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	swd_finish_read(dap);

	if (reg == DP_SELECT) {
		dap->select = data & (DP_SELECT_APSEL | DP_SELECT_APBANK | DP_SELECT_DPBANK);

		swd->write_reg(swd_cmd(false, false, reg), data, 0);

		retval = check_sync(dap);
		if (retval != ERROR_OK)
			dap->select = DP_SELECT_INVALID;

		return retval;
	}

	retval = swd_queue_dp_bankselect(dap, reg);
	if (retval != ERROR_OK)
		return retval;

	swd->write_reg(swd_cmd(false, false, reg), data, 0);

	return check_sync(dap);
}


static int swd_multidrop_select_inner(struct adiv5_dap *dap, uint32_t *dpidr_ptr,
		uint32_t *dlpidr_ptr, bool clear_sticky)
{
	int retval;
	uint32_t dpidr, dlpidr;

	assert(swd_dap_is_multidrop(dap));

	swd_send_sequence(dap, LINE_RESET);

	retval = swd_queue_dp_write_inner(dap, DP_TARGETSEL, dap->tap->multidrop_targetsel);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_queue_dp_read_inner(dap, DP_DPIDR, &dpidr);
	if (retval != ERROR_OK)
		return retval;

	if (clear_sticky) {
		/* Clear all sticky errors (including ORUN) */
		swd_clear_sticky_errors(dap);
	} else {
		/* Ideally just clear ORUN flag which is set by reset */
		retval = swd_queue_dp_write_inner(dap, DP_ABORT, ORUNERRCLR);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = swd_queue_dp_read_inner(dap, DP_DLPIDR, &dlpidr);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_run_inner(dap);
	if (retval != ERROR_OK)
		return retval;

	if ((dpidr & DP_DPIDR_VERSION_MASK) < (2UL << DP_DPIDR_VERSION_SHIFT)) {
		LOG_INFO("Read DPIDR 0x%08" PRIx32
				 " has version < 2. A non multidrop capable device connected?",
				 dpidr);
		return ERROR_FAIL;
	}

	/* TODO: check TARGETID if DLIPDR is same for more than one DP */
	uint32_t expected_dlpidr = DP_DLPIDR_PROTVSN |
			(dap->tap->multidrop_targetsel & DP_TARGETSEL_INSTANCEID_MASK);
	if (dlpidr != expected_dlpidr) {
		LOG_INFO("Read incorrect DLPIDR 0x%08" PRIx32
				 " (possibly CTRL/STAT value)",
				 dlpidr);
		return ERROR_FAIL;
	}

	LOG_DEBUG_IO("Selected DP_TARGETSEL 0x%08" PRIx32, dap->tap->multidrop_targetsel);
	swd_multidrop_selected_dap = dap;

	if (dpidr_ptr)
		*dpidr_ptr = dpidr;

	if (dlpidr_ptr)
		*dlpidr_ptr = dlpidr;

	return retval;
}

static int swd_multidrop_select(struct adiv5_dap *dap)
{
	if (!swd_dap_is_multidrop(dap))
		return ERROR_OK;

	if (swd_multidrop_selected_dap == dap)
		return ERROR_OK;

	int retval = ERROR_OK;
	for (unsigned int retry = 0; ; retry++) {
		bool clear_sticky = retry > 0;

		retval = swd_multidrop_select_inner(dap, NULL, NULL, clear_sticky);
		if (retval == ERROR_OK)
			break;

		swd_multidrop_selected_dap = NULL;
		if (retry > 3) {
			LOG_ERROR("Failed to select multidrop %s", adiv5_dap_name(dap));
			return retval;
		}

		LOG_DEBUG("Failed to select multidrop %s, retrying...",
				  adiv5_dap_name(dap));
	}

	return retval;
}

static int swd_connect_multidrop(struct adiv5_dap *dap)
{
	int retval;
	uint32_t dpidr = 0xdeadbeef;
	uint32_t dlpidr = 0xdeadbeef;
	int64_t timeout = timeval_ms() + 500;

	do {
		swd_send_sequence(dap, JTAG_TO_DORMANT);
		swd_send_sequence(dap, DORMANT_TO_SWD);

		/* Clear link state, including the SELECT cache. */
		dap->do_reconnect = false;
		dap_invalidate_cache(dap);
		swd_multidrop_selected_dap = NULL;

		retval = swd_multidrop_select_inner(dap, &dpidr, &dlpidr, true);
		if (retval == ERROR_OK)
			break;

		alive_sleep(1);

	} while (timeval_ms() < timeout);

	if (retval != ERROR_OK) {
		swd_multidrop_selected_dap = NULL;
		LOG_ERROR("Failed to connect multidrop %s", adiv5_dap_name(dap));
		return retval;
	}

	LOG_INFO("SWD DPIDR 0x%08" PRIx32 ", DLPIDR 0x%08" PRIx32,
			  dpidr, dlpidr);

	return retval;
}

static int swd_connect_single(struct adiv5_dap *dap)
{
	int retval;
	uint32_t dpidr = 0xdeadbeef;
	int64_t timeout = timeval_ms() + 500;

	do {
		swd_send_sequence(dap, JTAG_TO_SWD);

		/* Clear link state, including the SELECT cache. */
		dap->do_reconnect = false;
		dap_invalidate_cache(dap);

		retval = swd_queue_dp_read_inner(dap, DP_DPIDR, &dpidr);
		if (retval == ERROR_OK) {
			retval = swd_run_inner(dap);
			if (retval == ERROR_OK)
				break;
		}

		alive_sleep(1);

	} while (timeval_ms() < timeout);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error connecting DP: cannot read IDR");
		return retval;
	}

	LOG_INFO("SWD DPIDR 0x%08" PRIx32, dpidr);

	do {
		dap->do_reconnect = false;

		/* force clear all sticky faults */
		swd_clear_sticky_errors(dap);

		retval = swd_run_inner(dap);
		if (retval != ERROR_WAIT)
			break;

		alive_sleep(10);

	} while (timeval_ms() < timeout);

	return retval;
}

static int swd_connect(struct adiv5_dap *dap)
{
	int status;

	/* FIXME validate transport config ... is the
	 * configured DAP present (check IDCODE)?
	 */

	/* Check if we should reset srst already when connecting, but not if reconnecting. */
	if (!dap->do_reconnect) {
		enum reset_types jtag_reset_config = jtag_get_reset_config();

		if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
			if (jtag_reset_config & RESET_SRST_NO_GATING)
				adapter_assert_reset();
			else
				LOG_WARNING("\'srst_nogate\' reset_config option is required");
		}
	}

	if (swd_dap_is_multidrop(dap))
		status = swd_connect_multidrop(dap);
	else
		status = swd_connect_single(dap);

	/* IHI 0031E B4.3.2:
	 * "A WAIT response must not be issued to the ...
	 * ... writes to the ABORT register"
	 * swd_clear_sticky_errors() writes to the ABORT register only.
	 *
	 * Unfortunately at least Microchip SAMD51/E53/E54 returns WAIT
	 * in a corner case. Just try if ABORT resolves the problem.
	 */
	if (status == ERROR_WAIT) {
		LOG_WARNING("Connecting DP: stalled AP operation, issuing ABORT");

		dap->do_reconnect = false;

		status = swd_queue_dp_write_inner(dap, DP_ABORT,
			DAPABORT | STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);

		if (status == ERROR_OK)
			status = swd_run_inner(dap);
	}

	if (status == ERROR_OK)
		status = dap_dp_init(dap);

	return status;
}

static int swd_check_reconnect(struct adiv5_dap *dap)
{
	if (dap->do_reconnect)
		return swd_connect(dap);

	return ERROR_OK;
}

static int swd_queue_ap_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	/* TODO: Send DAPABORT in swd_multidrop_select_inner()
	 * in the case the multidrop dap is not selected?
	 * swd_queue_ap_abort() is not currently used anyway...
	 */
	int retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	swd->write_reg(swd_cmd(false, false, DP_ABORT),
		DAPABORT | STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
	return check_sync(dap);
}

static int swd_queue_dp_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	int retval = swd_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	return swd_queue_dp_read_inner(dap, reg, data);
}

static int swd_queue_dp_write(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	int retval = swd_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	return swd_queue_dp_write_inner(dap, reg, data);
}

/** Select the AP register bank matching bits 7:4 of reg. */
static int swd_queue_ap_bankselect(struct adiv5_ap *ap, unsigned reg)
{
	struct adiv5_dap *dap = ap->dap;
	uint32_t sel = ((uint32_t)ap->ap_num << 24)
			| (reg & 0x000000F0)
			| (dap->select & DP_SELECT_DPBANK);

	if (sel == dap->select)
		return ERROR_OK;

	dap->select = sel;

	int retval = swd_queue_dp_write_inner(dap, DP_SELECT, sel);
	if (retval != ERROR_OK)
		dap->select = DP_SELECT_INVALID;

	return retval;
}

static int swd_queue_ap_read(struct adiv5_ap *ap, unsigned reg,
		uint32_t *data)
{
	struct adiv5_dap *dap = ap->dap;
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	int retval = swd_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_queue_ap_bankselect(ap, reg);
	if (retval != ERROR_OK)
		return retval;

	swd->read_reg(swd_cmd(true, true, reg), dap->last_read, ap->memaccess_tck);
	dap->last_read = data;

	return check_sync(dap);
}

static int swd_queue_ap_write(struct adiv5_ap *ap, unsigned reg,
		uint32_t data)
{
	struct adiv5_dap *dap = ap->dap;
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	int retval = swd_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	swd_finish_read(dap);

	retval = swd_queue_ap_bankselect(ap, reg);
	if (retval != ERROR_OK)
		return retval;

	swd->write_reg(swd_cmd(false, true, reg), data, ap->memaccess_tck);

	return check_sync(dap);
}

/** Executes all queued DAP operations. */
static int swd_run(struct adiv5_dap *dap)
{
	int retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	swd_finish_read(dap);

	return swd_run_inner(dap);
}

/** Put the SWJ-DP back to JTAG mode */
static void swd_quit(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	static bool done;

	/* There is no difference if the sequence is sent at the last
	 * or the first swd_quit() call, send it just once */
	if (done)
		return;

	done = true;
	if (swd_dap_is_multidrop(dap))
		swd->switch_seq(SWD_TO_DORMANT);
	else
		swd->switch_seq(SWD_TO_JTAG);

	/* flush the queue to shift out the sequence before exit */
	swd->run();
}

const struct dap_ops swd_dap_ops = {
	.connect = swd_connect,
	.send_sequence = swd_send_sequence,
	.queue_dp_read = swd_queue_dp_read,
	.queue_dp_write = swd_queue_dp_write,
	.queue_ap_read = swd_queue_ap_read,
	.queue_ap_write = swd_queue_ap_write,
	.queue_ap_abort = swd_queue_ap_abort,
	.run = swd_run,
	.quit = swd_quit,
};

static const struct command_registration swd_commands[] = {
	{
		/*
		 * Set up SWD and JTAG targets identically, unless/until
		 * infrastructure improves ...  meanwhile, ignore all
		 * JTAG-specific stuff like IR length for SWD.
		 *
		 * REVISIT can we verify "just one SWD DAP" here/early?
		 */
		.name = "newdap",
		.jim_handler = jim_jtag_newtap,
		.mode = COMMAND_CONFIG,
		.help = "declare a new SWD DAP"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration swd_handlers[] = {
	{
		.name = "swd",
		.mode = COMMAND_ANY,
		.help = "SWD command group",
		.chain = swd_commands,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int swd_select(struct command_context *ctx)
{
	/* FIXME: only place where global 'adapter_driver' is still needed */
	extern struct adapter_driver *adapter_driver;
	const struct swd_driver *swd = adapter_driver->swd_ops;
	int retval;

	retval = register_commands(ctx, NULL, swd_handlers);
	if (retval != ERROR_OK)
		return retval;

	 /* be sure driver is in SWD mode; start
	  * with hardware default TRN (1), it can be changed later
	  */
	if (!swd || !swd->read_reg || !swd->write_reg || !swd->init) {
		LOG_DEBUG("no SWD driver?");
		return ERROR_FAIL;
	}

	retval = swd->init();
	if (retval != ERROR_OK) {
		LOG_DEBUG("can't init SWD driver");
		return retval;
	}

	return retval;
}

static int swd_init(struct command_context *ctx)
{
	/* nothing done here, SWD is initialized
	 * together with the DAP */
	return ERROR_OK;
}

static struct transport swd_transport = {
	.name = "swd",
	.select = swd_select,
	.init = swd_init,
};

static void swd_constructor(void) __attribute__((constructor));
static void swd_constructor(void)
{
	transport_register(&swd_transport);
}

/** Returns true if the current debug session
 * is using SWD as its transport.
 */
bool transport_is_swd(void)
{
	return get_current_transport() == &swd_transport;
}
