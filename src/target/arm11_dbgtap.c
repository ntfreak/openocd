/***************************************************************************
 *   Copyright (C) 2008 digenius technology GmbH.                          *
 *   Michael Bruck                                                         *
 *                                                                         *
 *   Copyright (C) 2008,2009 Oyvind Harboe oyvind.harboe@zylin.com         *
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

#include "arm_jtag.h"
#include "arm11_dbgtap.h"

#include <helper/time_support.h>

#if 0
#define JTAG_DEBUG(expr ...)    do { if (1) \
					     LOG_DEBUG(expr); } while (0)
#else
#define JTAG_DEBUG(expr ...)    do { if (0) \
					     LOG_DEBUG(expr); } while (0)
#endif

/*
This pathmove goes from Pause-IR to Shift-IR while avoiding RTI. The
behavior of the FTDI driver IIRC was to go via RTI.

Conversely there may be other places in this code where the ARM11 code relies
on the driver to hit through RTI when coming from Update-?R.
*/
static const tap_state_t arm11_move_pi_to_si_via_ci[] = {
	TAP_IREXIT2, TAP_IRUPDATE, TAP_DRSELECT, TAP_IRSELECT, TAP_IRCAPTURE, TAP_IRSHIFT
};

/* REVISIT no error handling here! */
static void arm11_add_ir_scan_vc(struct jtag_tap *tap, struct scan_field *fields,
	tap_state_t state)
{
	if (cmd_queue_cur_state == TAP_IRPAUSE)
		jtag_add_pathmove(ARRAY_SIZE(arm11_move_pi_to_si_via_ci),
			arm11_move_pi_to_si_via_ci);

	jtag_add_ir_scan(tap, fields, state);
}

static const tap_state_t arm11_move_pd_to_sd_via_cd[] = {
	TAP_DREXIT2, TAP_DRUPDATE, TAP_DRSELECT, TAP_DRCAPTURE, TAP_DRSHIFT
};

/* REVISIT no error handling here! */
void arm11_add_dr_scan_vc(struct jtag_tap *tap, int num_fields, struct scan_field *fields,
	tap_state_t state)
{
	if (cmd_queue_cur_state == TAP_DRPAUSE)
		jtag_add_pathmove(ARRAY_SIZE(arm11_move_pd_to_sd_via_cd),
			arm11_move_pd_to_sd_via_cd);

	jtag_add_dr_scan(tap, num_fields, fields, state);
}


/** Code de-clutter: Construct struct scan_field to write out a value
 *
 * \param arm11			Target state variable.
 * \param num_bits		Length of the data field
 * \param out_data		pointer to the data that will be sent out
 *						<em > (data is read when it is added to the JTAG queue)</em>
 * \param in_data		pointer to the memory that will receive data that was clocked in
 *						<em > (data is written when the JTAG queue is executed)</em>
 * \param field			target data structure that will be initialized
 */
void arm11_setup_field(struct arm11_common *arm11, int num_bits,
	void *out_data, void *in_data, struct scan_field *field)
{
	field->num_bits                 = num_bits;
	field->out_value                = out_data;
	field->in_value                 = in_data;
}

static const char *arm11_ir_to_string(uint8_t ir)
{
	const char *s = "unknown";

	switch (ir) {
		case ARM11_EXTEST:
			s = "EXTEST";
			break;
		case ARM11_SCAN_N:
			s = "SCAN_N";
			break;
		case ARM11_RESTART:
			s = "RESTART";
			break;
		case ARM11_HALT:
			s = "HALT";
			break;
		case ARM11_INTEST:
			s = "INTEST";
			break;
		case ARM11_ITRSEL:
			s = "ITRSEL";
			break;
		case ARM11_IDCODE:
			s = "IDCODE";
			break;
		case ARM11_BYPASS:
			s = "BYPASS";
			break;
	}
	return s;
}

/** Write JTAG instruction register
 *
 * \param arm11		Target state variable.
 * \param instr		An ARM11 DBGTAP instruction. Use enum #arm11_instructions.
 * \param state		Pass the final TAP state or ARM11_TAP_DEFAULT for the default value (Pause-IR).
 *
 * \remarks			This adds to the JTAG command queue but does \em not execute it.
 */
void arm11_add_ir(struct arm11_common *arm11, uint8_t instr, tap_state_t state)
{
	struct jtag_tap *tap = arm11->arm.target->tap;

	if (buf_get_u32(tap->cur_instr, 0, 5) == instr) {
		JTAG_DEBUG("IR <= 0x%02x SKIPPED", instr);
		return;
	}

	JTAG_DEBUG("IR <= %s (0x%02x)", arm11_ir_to_string(instr), instr);

	struct scan_field field;

	arm11_setup_field(arm11, 5, &instr, NULL, &field);

	arm11_add_ir_scan_vc(arm11->arm.target->tap,
		&field,
		state == ARM11_TAP_DEFAULT ? TAP_IRPAUSE : state);
}

/** Verify data shifted out from Scan Chain Register (SCREG). */
static void arm11_in_handler_scan_n(uint8_t *in_value)
{
	/* Don't expect JTAG layer to modify bits we didn't ask it to read */
	uint8_t v = *in_value & 0x1F;

	if (v != 0x10) {
		LOG_ERROR("'arm11 target' JTAG error SCREG OUT 0x%02x", v);
		jtag_set_error(ERROR_FAIL);
	}
}

/** Select and write to Scan Chain Register (SCREG)
 *
 * This function sets the instruction register to SCAN_N and writes
 * the data register with the selected chain number.
 *
 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0301f/Cacbjhfg.html
 *
 * \param arm11	    Target state variable.
 * \param chain	    Scan chain that will be selected.
 * \param state	    Pass the final TAP state or ARM11_TAP_DEFAULT for the default
 *					value (Pause-DR).
 *
 * Changes the current scan chain if needed, transitions to the specified
 * TAP state, and leaves the IR undefined.
 *
 * The chain takes effect when Update-DR is passed (usually when subsequently
 * the INTEXT/EXTEST instructions are written).
 *
 * \warning			(Obsolete) Using this twice in a row will \em fail. The first
 *					call will end in Pause-DR. The second call, due to the IR
 *					caching, will not go through Capture-DR when shifting in the
 *					new scan chain number. As a result the verification in
 *					arm11_in_handler_scan_n() must fail.
 *
 * \remarks			This adds to the JTAG command queue but does \em not execute it.
 */

int arm11_add_debug_scan_n(struct arm11_common *arm11,
	uint8_t chain, tap_state_t state)
{
	/* Don't needlessly switch the scan chain.
	 * NOTE:  the ITRSEL instruction fakes SCREG changing;
	 * but leaves its actual value unchanged.
	 */
#if 0
	/* FIX!!! the optimization below is broken because we do not */
	/* invalidate the cur_scan_chain upon a TRST/TMS. See arm_jtag.c */
	/* for example on how to invalidate cur_scan_chain. Tested patches gladly */
	/* accepted! */
	if (arm11->jtag_info.cur_scan_chain == chain) {
		JTAG_DEBUG("SCREG <= %d SKIPPED", chain);
		return jtag_add_statemove((state == ARM11_TAP_DEFAULT)
			? TAP_DRPAUSE : state);
	}
#endif
	JTAG_DEBUG("SCREG <= %d", chain);

	arm11_add_ir(arm11, ARM11_SCAN_N, ARM11_TAP_DEFAULT);

	struct scan_field field;

	uint8_t tmp[1];
	arm11_setup_field(arm11, 5, &chain, &tmp, &field);

	arm11_add_dr_scan_vc(arm11->arm.target->tap,
		1,
		&field,
		state == ARM11_TAP_DEFAULT ? TAP_DRPAUSE : state);

	jtag_execute_queue_noclear();

	arm11_in_handler_scan_n(tmp);

	arm11->jtag_info.cur_scan_chain = chain;

	return jtag_execute_queue();
}

/**
 * Queue a DR scan of the ITR register.  Caller must have selected
 * scan chain 4 (ITR), possibly using ITRSEL.
 *
 * \param arm11		Target state variable.
 * \param inst		An ARM11 processor instruction/opcode.
 * \param flag		Optional parameter to retrieve the Ready flag;
 *	this address will be written when the JTAG chain is scanned.
 * \param state		The TAP state to enter after the DR scan.
 *
 * Going through the TAP_DRUPDATE state writes ITR only if Ready was
 * previously set.  Only the Ready flag is readable by the scan.
 *
 * An instruction loaded into ITR is executed when going through the
 * TAP_IDLE state only if Ready was previously set and the debug state
 * is properly set up.  Depending on the instruction, you may also need
 * to ensure that the rDTR is ready before that Run-Test/Idle state.
 */
static void arm11_add_debug_inst(struct arm11_common *arm11,
	uint32_t inst, uint8_t *flag, tap_state_t state)
{
	JTAG_DEBUG("INST <= 0x%08x", (unsigned) inst);

	struct scan_field itr[2];

	arm11_setup_field(arm11, 32, &inst, NULL, itr + 0);
	arm11_setup_field(arm11, 1, NULL, flag, itr + 1);

	arm11_add_dr_scan_vc(arm11->arm.target->tap, ARRAY_SIZE(itr), itr, state);
}

/**
 * Read and save the Debug Status and Control Register (DSCR).
 *
 * \param arm11		Target state variable.
 * \return Error status; arm11->dscr is updated on success.
 *
 * \remarks This is a stand-alone function that executes the JTAG
 * command queue.  It does not require the ARM11 debug TAP to be
 * in any particular state.
 */
int arm11_read_dscr(struct arm11_common *arm11)
{
	int retval;

	retval = arm11_add_debug_scan_n(arm11, 0x01, ARM11_TAP_DEFAULT);
	if (retval != ERROR_OK)
		return retval;

	arm11_add_ir(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	uint32_t dscr;
	struct scan_field chain1_field;

	arm11_setup_field(arm11, 32, NULL, &dscr, &chain1_field);

	arm11_add_dr_scan_vc(arm11->arm.target->tap, 1, &chain1_field, TAP_DRPAUSE);

	CHECK_RETVAL(jtag_execute_queue());

	if (arm11->dscr != dscr)
		JTAG_DEBUG("DSCR  = %08x (OLD %08x)",
			(unsigned) dscr,
			(unsigned) arm11->dscr);

	arm11->dscr = dscr;

	return ERROR_OK;
}

/** Write the Debug Status and Control Register (DSCR)
 *
 * same as CP14 c1
 *
 * \param arm11		Target state variable.
 * \param dscr		DSCR content
 *
 * \remarks			This is a stand-alone function that executes the JTAG command queue.
 */
int arm11_write_dscr(struct arm11_common *arm11, uint32_t dscr)
{
	int retval;
	retval = arm11_add_debug_scan_n(arm11, 0x01, ARM11_TAP_DEFAULT);
	if (retval != ERROR_OK)
		return retval;

	arm11_add_ir(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

	struct scan_field chain1_field;

	arm11_setup_field(arm11, 32, &dscr, NULL, &chain1_field);

	arm11_add_dr_scan_vc(arm11->arm.target->tap, 1, &chain1_field, TAP_DRPAUSE);

	CHECK_RETVAL(jtag_execute_queue());

	JTAG_DEBUG("DSCR <= %08x (OLD %08x)",
		(unsigned) dscr,
		(unsigned) arm11->dscr);

	arm11->dscr = dscr;

	return ERROR_OK;
}

/** Prepare the stage for ITR/DTR operations
 * from the arm11_run_instr... group of functions.
 *
 * Put arm11_run_instr_data_prepare() and arm11_run_instr_data_finish()
 * around a block of arm11_run_instr_... calls.
 *
 * Select scan chain 5 to allow quick access to DTR. When scan
 * chain 4 is needed to put in a register the ITRSel instruction
 * shortcut is used instead of actually changing the Scan_N
 * register.
 *
 * \param arm11		Target state variable.
 *
 */
int arm11_run_instr_data_prepare(struct arm11_common *arm11)
{
	return arm11_add_debug_scan_n(arm11, 0x05, ARM11_TAP_DEFAULT);
}

/** Cleanup after ITR/DTR operations
 * from the arm11_run_instr... group of functions
 *
 * Put arm11_run_instr_data_prepare() and arm11_run_instr_data_finish()
 * around a block of arm11_run_instr_... calls.
 *
 * Any IDLE can lead to an instruction execution when
 * scan chains 4 or 5 are selected and the IR holds
 * INTEST or EXTEST. So we must disable that before
 * any following activities lead to an IDLE.
 *
 * \param arm11		Target state variable.
 *
 */
int arm11_run_instr_data_finish(struct arm11_common *arm11)
{
	return arm11_add_debug_scan_n(arm11, 0x00, ARM11_TAP_DEFAULT);
}

/**
 * Execute one or more instructions via ITR.
 * Caller guarantees that processor is in debug state, that DSCR_ITR_EN
 * is set, the ITR Ready flag is set (as seen on the previous entry to
 * TAP_DRCAPTURE), and the DSCR sticky abort flag is clear.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	Pointer to sequence of ARM opcodes
 * \param count		Number of opcodes to execute
 *
 */
static
int arm11_run_instr_no_data(struct arm11_common *arm11,
	uint32_t *opcode, size_t count)
{
	arm11_add_ir(arm11, ARM11_ITRSEL, ARM11_TAP_DEFAULT);

	while (count--) {
		arm11_add_debug_inst(arm11, *opcode++, NULL, TAP_IDLE);

		int i = 0;
		while (1) {
			uint8_t flag;

			arm11_add_debug_inst(arm11, 0, &flag, count ? TAP_IDLE : TAP_DRPAUSE);

			CHECK_RETVAL(jtag_execute_queue());

			if (flag)
				break;

			int64_t then = 0;

			if (i == 1000)
				then = timeval_ms();
			if (i >= 1000) {
				if ((timeval_ms()-then) > 1000) {
					LOG_WARNING(
						"Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}

			i++;
		}
	}

	return ERROR_OK;
}

/** Execute one instruction via ITR
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 *
 */
int arm11_run_instr_no_data1(struct arm11_common *arm11, uint32_t opcode)
{
	return arm11_run_instr_no_data(arm11, &opcode, 1);
}


/** Execute one instruction via ITR repeatedly while
 *  passing data to the core via DTR on each execution.
 *
 * Caller guarantees that processor is in debug state, that DSCR_ITR_EN
 * is set, the ITR Ready flag is set (as seen on the previous entry to
 * TAP_DRCAPTURE), and the DSCR sticky abort flag is clear.
 *
 *  The executed instruction \em must read data from DTR.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 * \param data		Pointer to the data words to be passed to the core
 * \param count		Number of data words and instruction repetitions
 *
 */
int arm11_run_instr_data_to_core(struct arm11_common *arm11,
	uint32_t opcode,
	uint32_t *data,
	size_t count)
{
	arm11_add_ir(arm11, ARM11_ITRSEL, ARM11_TAP_DEFAULT);

	arm11_add_debug_inst(arm11, opcode, NULL, TAP_DRPAUSE);

	arm11_add_ir(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

	struct scan_field chain5_fields[3];

	uint32_t _data;
	uint8_t ready;
	uint8_t n_retry;

	arm11_setup_field(arm11, 32,    &_data, NULL,           chain5_fields + 0);
	arm11_setup_field(arm11,  1,    NULL,   &ready,         chain5_fields + 1);
	arm11_setup_field(arm11,  1,    NULL,   &n_retry,       chain5_fields + 2);

	while (count--) {
		int i = 0;
		do {
			_data        = *data;

			arm11_add_dr_scan_vc(arm11->arm.target->tap, ARRAY_SIZE(
					chain5_fields), chain5_fields, TAP_IDLE);

			CHECK_RETVAL(jtag_execute_queue());

			JTAG_DEBUG("DTR  ready %d  n_retry %d", ready, n_retry);

			int64_t then = 0;

			if (i == 1000)
				then = timeval_ms();
			if (i >= 1000) {
				if ((timeval_ms()-then) > 1000) {
					LOG_WARNING(
						"Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}

			i++;
		} while (!ready);

		data++;
	}

	arm11_add_ir(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	int i = 0;
	do {
		_data        = 0;

		arm11_add_dr_scan_vc(arm11->arm.target->tap, ARRAY_SIZE(
				chain5_fields), chain5_fields, TAP_DRPAUSE);

		CHECK_RETVAL(jtag_execute_queue());

		JTAG_DEBUG("DTR  _data %08x  ready %d  n_retry %d",
			(unsigned) _data, ready, n_retry);

		int64_t then = 0;

		if (i == 1000)
			then = timeval_ms();
		if (i >= 1000) {
			if ((timeval_ms()-then) > 1000) {
				LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
				return ERROR_FAIL;
			}
		}

		i++;
	} while (!ready);

	return ERROR_OK;
}

/** JTAG path for arm11_run_instr_data_to_core_noack
 *
 *  The repeated TAP_IDLE's do not cause a repeated execution
 *  if passed without leaving the state.
 *
 *  Since this is more than 7 bits (adjustable via adding more
 *  TAP_IDLE's) it produces an artificial delay in the lower
 *  layer (FT2232) that is long enough to finish execution on
 *  the core but still shorter than any manually inducible delays.
 *
 *  To disable this code, try "memwrite burst false"
 *
 *  FIX!!! should we use multiple TAP_IDLE here or not???
 *
 *  https://lists.berlios.de/pipermail/openocd-development/2009-July/009698.html
 *  https://lists.berlios.de/pipermail/openocd-development/2009-August/009865.html
 */
static const tap_state_t arm11_move_drpause_idle_drpause_with_delay[] = {
	TAP_DREXIT2, TAP_DRUPDATE, TAP_IDLE, TAP_IDLE, TAP_IDLE, TAP_DRSELECT, TAP_DRCAPTURE,
	TAP_DRSHIFT
};

static int arm11_run_instr_data_to_core_noack_inner(struct jtag_tap *tap,
	uint32_t opcode,
	uint32_t *data,
	size_t count)
{
	struct scan_field chain5_fields[3];

	chain5_fields[0].num_bits               = 32;
	chain5_fields[0].out_value              = NULL;	/*&Data*/
	chain5_fields[0].in_value               = NULL;

	chain5_fields[1].num_bits               = 1;
	chain5_fields[1].out_value              = NULL;
	chain5_fields[1].in_value               = NULL;	/*&Ready*/

	chain5_fields[2].num_bits               = 1;
	chain5_fields[2].out_value              = NULL;
	chain5_fields[2].in_value               = NULL;

	uint8_t *readies;
	unsigned readies_num = count;
	unsigned bytes = sizeof(*readies)*readies_num;

	readies = malloc(bytes);
	if (!readies) {
		LOG_ERROR("Out of memory allocating %u bytes", bytes);
		return ERROR_FAIL;
	}

	uint8_t *ready_pos                      = readies;
	while (count--) {
		chain5_fields[0].out_value      = (uint8_t *)(data++);
		chain5_fields[1].in_value       = ready_pos++;

		if (count > 0) {
			jtag_add_dr_scan(tap, ARRAY_SIZE(chain5_fields), chain5_fields,
				TAP_DRPAUSE);
			jtag_add_pathmove(ARRAY_SIZE(arm11_move_drpause_idle_drpause_with_delay),
				arm11_move_drpause_idle_drpause_with_delay);
		} else
			jtag_add_dr_scan(tap, ARRAY_SIZE(chain5_fields), chain5_fields, TAP_IDLE);
	}

	int retval = jtag_execute_queue();
	if (retval == ERROR_OK) {
		unsigned error_count = 0;

		for (size_t i = 0; i < readies_num; i++) {
			if (readies[i] != 1)
				error_count++;
		}

		if (error_count > 0) {
			LOG_ERROR("%u words out of %u not transferred",
				error_count, readies_num);
			retval = ERROR_FAIL;
		}
	}
	free(readies);

	return retval;
}

/** Execute one instruction via ITR repeatedly while
 *  passing data to the core via DTR on each execution.
 *
 * Caller guarantees that processor is in debug state, that DSCR_ITR_EN
 * is set, the ITR Ready flag is set (as seen on the previous entry to
 * TAP_DRCAPTURE), and the DSCR sticky abort flag is clear.
 *
 *  No Ready check during transmission.
 *
 *  The executed instruction \em must read data from DTR.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 * \param data		Pointer to the data words to be passed to the core
 * \param count		Number of data words and instruction repetitions
 *
 */
int arm11_run_instr_data_to_core_noack(struct arm11_common *arm11,
	uint32_t opcode,
	uint32_t *data,
	size_t count)
{
	arm11_add_ir(arm11, ARM11_ITRSEL, ARM11_TAP_DEFAULT);

	arm11_add_debug_inst(arm11, opcode, NULL, TAP_DRPAUSE);

	arm11_add_ir(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

	int retval = arm11_run_instr_data_to_core_noack_inner(arm11->arm.target->tap,
			opcode,
			data,
			count);

	if (retval != ERROR_OK)
		return retval;

	arm11_add_ir(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	struct scan_field chain5_fields[3];

	arm11_setup_field(arm11,
		32,
		NULL /*&Data*/,
		NULL,
		chain5_fields + 0);
	arm11_setup_field(arm11,
		1,
		NULL,
		NULL /*&Ready*/,
		chain5_fields + 1);
	arm11_setup_field(arm11,
		1,
		NULL,
		NULL,
		chain5_fields + 2);

	uint8_t ready_flag;
	chain5_fields[1].in_value   = &ready_flag;

	arm11_add_dr_scan_vc(arm11->arm.target->tap, ARRAY_SIZE(
			chain5_fields), chain5_fields, TAP_DRPAUSE);

	retval = jtag_execute_queue();
	if (retval == ERROR_OK) {
		if (ready_flag != 1) {
			LOG_ERROR("last word not transferred");
			retval = ERROR_FAIL;
		}
	}

	return retval;
}


/** Execute an instruction via ITR while handing data into the core via DTR.
 *
 *  The executed instruction \em must read data from DTR.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 * \param data		Data word to be passed to the core via DTR
 *
 */
int arm11_run_instr_data_to_core1(struct arm11_common *arm11, uint32_t opcode, uint32_t data)
{
	return arm11_run_instr_data_to_core(arm11, opcode, &data, 1);
}


/** Execute one instruction via ITR repeatedly while
 *  reading data from the core via DTR on each execution.
 *
 * Caller guarantees that processor is in debug state, that DSCR_ITR_EN
 * is set, the ITR Ready flag is set (as seen on the previous entry to
 * TAP_DRCAPTURE), and the DSCR sticky abort flag is clear.
 *
 *  The executed instruction \em must write data to DTR.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 * \param data		Pointer to an array that receives the data words from the core
 * \param count		Number of data words and instruction repetitions
 *
 */
int arm11_run_instr_data_from_core(struct arm11_common *arm11,
	uint32_t opcode,
	uint32_t *data,
	size_t count)
{
	arm11_add_ir(arm11, ARM11_ITRSEL, ARM11_TAP_DEFAULT);

	arm11_add_debug_inst(arm11, opcode, NULL, TAP_IDLE);

	arm11_add_ir(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	struct scan_field chain5_fields[3];

	uint32_t _data;
	uint8_t ready;
	uint8_t n_retry;

	arm11_setup_field(arm11, 32,    NULL,   &_data,     chain5_fields + 0);
	arm11_setup_field(arm11,  1,    NULL,   &ready,     chain5_fields + 1);
	arm11_setup_field(arm11,  1,    NULL,   &n_retry,   chain5_fields + 2);

	while (count--) {
		int i = 0;
		do {
			arm11_add_dr_scan_vc(arm11->arm.target->tap, ARRAY_SIZE(
					chain5_fields), chain5_fields,
				count ? TAP_IDLE : TAP_DRPAUSE);

			CHECK_RETVAL(jtag_execute_queue());

			JTAG_DEBUG("DTR  _data %08x  ready %d  n_retry %d",
				(unsigned) _data, ready, n_retry);

			int64_t then = 0;

			if (i == 1000)
				then = timeval_ms();
			if (i >= 1000) {
				if ((timeval_ms()-then) > 1000) {
					LOG_WARNING(
						"Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}

			i++;
		} while (!ready);

		*data++ = _data;
	}

	return ERROR_OK;
}

/** Execute one instruction via ITR
 *  then load r0 into DTR and read DTR from core.
 *
 *  The first executed instruction (\p opcode) should write data to r0.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode to write r0 with the value of interest
 * \param data		Pointer to a data word that receives the value from r0 after \p opcode was executed.
 *
 */
int arm11_run_instr_data_from_core_via_r0(struct arm11_common *arm11,
	uint32_t opcode,
	uint32_t *data)
{
	int retval;
	retval = arm11_run_instr_no_data1(arm11, opcode);
	if (retval != ERROR_OK)
		return retval;

	/* MCR p14,0,R0,c0,c5,0 (move r0 -> wDTR -> local var) */
	arm11_run_instr_data_from_core(arm11, 0xEE000E15, data, 1);

	return ERROR_OK;
}

/** Load data into core via DTR then move it to r0 then
 *  execute one instruction via ITR
 *
 *  The final executed instruction (\p opcode) should read data from r0.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode to read r0 act upon it
 * \param data		Data word that will be written to r0 before \p opcode is executed
 *
 */
int arm11_run_instr_data_to_core_via_r0(struct arm11_common *arm11, uint32_t opcode, uint32_t data)
{
	int retval;
	/* MRC p14,0,r0,c0,c5,0 */
	retval = arm11_run_instr_data_to_core1(arm11, 0xEE100E15, data);
	if (retval != ERROR_OK)
		return retval;

	retval = arm11_run_instr_no_data1(arm11, opcode);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/** Apply reads and writes to scan chain 7
 *
 * \see struct arm11_sc7_action
 *
 * \param arm11		Target state variable.
 * \param actions	A list of read and/or write instructions
 * \param count		Number of instructions in the list.
 *
 */
int arm11_sc7_run(struct arm11_common *arm11, struct arm11_sc7_action *actions, size_t count)
{
	int retval;

	retval = arm11_add_debug_scan_n(arm11, 0x07, ARM11_TAP_DEFAULT);
	if (retval != ERROR_OK)
		return retval;

	arm11_add_ir(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

	struct scan_field chain7_fields[3];

	uint8_t n_rw;
	uint32_t data_out;
	uint8_t address_out;
	uint8_t ready;
	uint32_t data_in;
	uint8_t address_in;

	arm11_setup_field(arm11,  1, &n_rw,              &ready,          chain7_fields + 0);
	arm11_setup_field(arm11, 32, &data_out,          &data_in,        chain7_fields + 1);
	arm11_setup_field(arm11,  7, &address_out,       &address_in,     chain7_fields + 2);

	for (size_t i = 0; i < count + 1; i++) {
		if (i < count) {
			n_rw             = actions[i].write ? 1 : 0;
			data_out         = actions[i].value;
			address_out      = actions[i].address;
		} else {
			n_rw             = 1;
			data_out         = 0;
			address_out      = 0;
		}

		/* Timeout here so we don't get stuck. */
		int i_n = 0;
		while (1) {
			JTAG_DEBUG("SC7 <= c%-3d Data %08x %s",
				(unsigned) address_out,
				(unsigned) data_out,
				n_rw ? "write" : "read");

			arm11_add_dr_scan_vc(arm11->arm.target->tap, ARRAY_SIZE(chain7_fields),
				chain7_fields, TAP_DRPAUSE);

			CHECK_RETVAL(jtag_execute_queue());

			/* 'n_rw' is 'ready' on read out */
			if (ready)
				break;

			int64_t then = 0;

			if (i_n == 1000)
				then = timeval_ms();
			if (i_n >= 1000) {
				if ((timeval_ms()-then) > 1000) {
					LOG_WARNING(
						"Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}

			i_n++;
		}

		if (!n_rw)
			JTAG_DEBUG("SC7 => Data %08x", (unsigned) data_in);

		if (i > 0) {
			if (actions[i - 1].address != address_in)
				LOG_WARNING("Scan chain 7 shifted out unexpected address");

			if (!actions[i - 1].write)
				actions[i - 1].value = data_in;
			else {
				if (actions[i - 1].value != data_in)
					LOG_WARNING("Scan chain 7 shifted out unexpected data");
			}
		}
	}
	return ERROR_OK;
}

/** Clear VCR and all breakpoints and watchpoints via scan chain 7
 *
 * \param arm11		Target state variable.
 *
 */
int arm11_sc7_clear_vbw(struct arm11_common *arm11)
{
	size_t clear_bw_size = arm11->brp + 1;
	struct arm11_sc7_action *clear_bw = malloc(sizeof(struct arm11_sc7_action) * clear_bw_size);
	struct arm11_sc7_action *pos = clear_bw;

	for (size_t i = 0; i < clear_bw_size; i++) {
		clear_bw[i].write       = true;
		clear_bw[i].value       = 0;
	}

	for (size_t i = 0; i < arm11->brp; i++)
		(pos++)->address = ARM11_SC7_BCR0 + i;

	(pos++)->address = ARM11_SC7_VCR;

	int retval;
	retval = arm11_sc7_run(arm11, clear_bw, clear_bw_size);

	free(clear_bw);

	return retval;
}

/** Write VCR register
 *
 * \param arm11		Target state variable.
 * \param value		Value to be written
 */
int arm11_sc7_set_vcr(struct arm11_common *arm11, uint32_t value)
{
	struct arm11_sc7_action set_vcr;

	set_vcr.write           = true;
	set_vcr.address         = ARM11_SC7_VCR;
	set_vcr.value           = value;

	return arm11_sc7_run(arm11, &set_vcr, 1);
}

/** Read word from address
 *
 * \param arm11		Target state variable.
 * \param address	Memory address to be read
 * \param result	Pointer where to store result
 *
 */
int arm11_read_memory_word(struct arm11_common *arm11, uint32_t address, uint32_t *result)
{
	int retval;
	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* MRC p14,0,r0,c0,c5,0 (r0 = address) */
	CHECK_RETVAL(arm11_run_instr_data_to_core1(arm11, 0xee100e15, address));

	/* LDC p14,c5,[R0],#4 (DTR = [r0]) */
	CHECK_RETVAL(arm11_run_instr_data_from_core(arm11, 0xecb05e01, result, 1));

	return arm11_run_instr_data_finish(arm11);
}

/************************************************************************/

/*
 * ARM11 provider for the OpenOCD implementation of the standard
 * architectural ARM v6/v7 "Debug Programmer's Model" (DPM).
 */

static inline struct arm11_common *dpm_to_arm11(struct arm_dpm *dpm)
{
	return container_of(dpm, struct arm11_common, dpm);
}

static int arm11_dpm_prepare(struct arm_dpm *dpm)
{
	return arm11_run_instr_data_prepare(dpm_to_arm11(dpm));
}

static int arm11_dpm_finish(struct arm_dpm *dpm)
{
	return arm11_run_instr_data_finish(dpm_to_arm11(dpm));
}

static int arm11_dpm_instr_write_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	return arm11_run_instr_data_to_core(dpm_to_arm11(dpm),
		opcode, &data, 1);
}

static int arm11_dpm_instr_write_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	return arm11_run_instr_data_to_core_via_r0(dpm_to_arm11(dpm),
		opcode, data);
}

static int arm11_dpm_instr_read_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	return arm11_run_instr_data_from_core(dpm_to_arm11(dpm),
		opcode, data, 1);
}

static int arm11_dpm_instr_read_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	return arm11_run_instr_data_from_core_via_r0(dpm_to_arm11(dpm),
		opcode, data);
}

/* Because arm11_sc7_run() takes a vector of actions, we batch breakpoint
 * and watchpoint operations instead of running them right away.  Since we
 * pre-allocated our vector, we don't need to worry about space.
 */
static int arm11_bpwp_enable(struct arm_dpm *dpm, unsigned index_t,
	uint32_t addr, uint32_t control)
{
	struct arm11_common *arm11 = dpm_to_arm11(dpm);
	struct arm11_sc7_action *action;

	action = arm11->bpwp_actions + arm11->bpwp_n;

	/* Invariant:  this bp/wp is disabled.
	 * It also happens that the core is halted here, but for
	 * DPM-based cores we don't actually care about that.
	 */

	action[0].write = action[1].write = true;

	action[0].value = addr;
	action[1].value = control;

	switch (index_t) {
	    case 0 ... 15:
		    action[0].address = ARM11_SC7_BVR0 + index_t;
		    action[1].address = ARM11_SC7_BCR0 + index_t;
		    break;
	    case 16 ... 32:
		    index_t -= 16;
		    action[0].address = ARM11_SC7_WVR0 + index_t;
		    action[1].address = ARM11_SC7_WCR0 + index_t;
		    break;
	    default:
		    return ERROR_FAIL;
	}

	arm11->bpwp_n += 2;

	return ERROR_OK;
}

static int arm11_bpwp_disable(struct arm_dpm *dpm, unsigned index_t)
{
	struct arm11_common *arm11 = dpm_to_arm11(dpm);
	struct arm11_sc7_action *action;

	action = arm11->bpwp_actions + arm11->bpwp_n;

	action[0].write = true;
	action[0].value = 0;

	switch (index_t) {
	    case 0 ... 15:
		    action[0].address = ARM11_SC7_BCR0 + index_t;
		    break;
	    case 16 ... 32:
		    index_t -= 16;
		    action[0].address = ARM11_SC7_WCR0 + index_t;
		    break;
	    default:
		    return ERROR_FAIL;
	}

	arm11->bpwp_n += 1;

	return ERROR_OK;
}

/** Flush any pending breakpoint and watchpoint updates. */
int arm11_bpwp_flush(struct arm11_common *arm11)
{
	int retval;

	if (!arm11->bpwp_n)
		return ERROR_OK;

	retval = arm11_sc7_run(arm11, arm11->bpwp_actions, arm11->bpwp_n);
	arm11->bpwp_n = 0;

	return retval;
}

/** Set up high-level debug module utilities */
int arm11_dpm_init(struct arm11_common *arm11, uint32_t didr)
{
	struct arm_dpm *dpm = &arm11->dpm;
	int retval;

	dpm->arm = &arm11->arm;

	dpm->didr = didr;

	dpm->prepare = arm11_dpm_prepare;
	dpm->finish = arm11_dpm_finish;

	dpm->instr_write_data_dcc = arm11_dpm_instr_write_data_dcc;
	dpm->instr_write_data_r0 = arm11_dpm_instr_write_data_r0;

	dpm->instr_read_data_dcc = arm11_dpm_instr_read_data_dcc;
	dpm->instr_read_data_r0 = arm11_dpm_instr_read_data_r0;

	dpm->bpwp_enable = arm11_bpwp_enable;
	dpm->bpwp_disable = arm11_bpwp_disable;

	retval = arm_dpm_setup(dpm);
	if (retval != ERROR_OK)
		return retval;

	/* alloc enough to enable all breakpoints and watchpoints at once */
	arm11->bpwp_actions = calloc(2 * (dpm->nbp + dpm->nwp),
			sizeof(*arm11->bpwp_actions));
	if (!arm11->bpwp_actions)
		return ERROR_FAIL;

	retval = arm_dpm_initialize(dpm);
	if (retval != ERROR_OK)
		return retval;

	return arm11_bpwp_flush(arm11);
}

void arm11_dpm_deinit(struct arm11_common *arm11)
{
	struct arm_dpm *dpm = &arm11->dpm;

	free(arm11->bpwp_actions);
	arm_free_reg_cache(dpm->arm);
	free(dpm->dbp);
	free(dpm->dwp);
}
