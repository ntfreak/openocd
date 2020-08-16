/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 Peter Hettkamp                                     *
 *   peter.hettkamp@htp-tel.de                                             *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation. http://softplc.com            *
 *   Dick Hollenbeck <dick@softplc.com>                                    *
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

/* The specification for SVF is available here:
 * http://www.asset-intertech.com/support/svf.pdf
 * Below, this document is referred to as the "SVF spec".
 *
 * The specification for XSVF is available here:
 * http://www.xilinx.com/support/documentation/application_notes/xapp503.pdf
 * Below, this document is referred to as the "XSVF spec".
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "xsvf.h"
#include <jtag/jtag.h>
#include <svf/svf.h>

/* XSVF commands, from appendix B of xapp503.pdf  */
#define XCOMPLETE			0x00
#define XTDOMASK			0x01
#define XSIR				0x02
#define XSDR				0x03
#define XRUNTEST			0x04
#define XREPEAT				0x07
#define XSDRSIZE			0x08
#define XSDRTDO				0x09
#define XSETSDRMASKS		0x0A
#define XSDRINC				0x0B
#define XSDRB				0x0C
#define XSDRC				0x0D
#define XSDRE				0x0E
#define XSDRTDOB			0x0F
#define XSDRTDOC			0x10
#define XSDRTDOE			0x11
#define XSTATE				0x12
#define XENDIR				0x13
#define XENDDR				0x14
#define XSIR2				0x15
#define XCOMMENT			0x16
#define XWAIT				0x17

/* XWAITSTATE is not in the xilinx XSVF spec, but the svf2xsvf.py translator
 * generates this.  Arguably it is needed because the XSVF XRUNTEST command
 * was ill conceived and does not directly flow out of the SVF RUNTEST command.
 * This XWAITSTATE does map directly from the SVF RUNTEST command.
 */
#define XWAITSTATE			0x18

/* Lattice has extended the SVF file format, and Dick Hollenbeck's python based
 * SVF2XSVF converter supports these 3 additional XSVF opcodes, LCOUNT, LDELAY, LSDR.
 * Here is an example of usage of the 3 lattice opcode extensions:

! Set the maximum loop count to 25.
LCOUNT	25;
! Step to DRPAUSE give 5 clocks and wait for 1.00e + 000 SEC.
LDELAY	DRPAUSE	5 TCK	1.00E-003 SEC;
! Test for the completed status. Match means pass.
! Loop back to LDELAY line if not match and loop count less than 25.

LSDR 1  TDI  (0)
TDO  (1);
*/

#define LCOUNT				0x19
#define LDELAY				0x1A
#define LSDR				0x1B
#define XTRST				0x1C

/* XSVF valid state values for the XSTATE command, from appendix B of xapp503.pdf */
#define XSV_RESET			0x00
#define XSV_IDLE			0x01
#define XSV_DRSELECT		0x02
#define XSV_DRCAPTURE		0x03
#define XSV_DRSHIFT			0x04
#define XSV_DREXIT1			0x05
#define XSV_DRPAUSE			0x06
#define XSV_DREXIT2			0x07
#define XSV_DRUPDATE		0x08
#define XSV_IRSELECT		0x09
#define XSV_IRCAPTURE		0x0A
#define XSV_IRSHIFT			0x0B
#define XSV_IREXIT1			0x0C
#define XSV_IRPAUSE			0x0D
#define XSV_IREXIT2			0x0E
#define XSV_IRUPDATE		0x0F

/* arguments to XTRST */
#define XTRST_ON			0
#define XTRST_OFF			1
#define XTRST_Z				2
#define XTRST_ABSENT		3

#define XSTATE_MAX_PATH 12

static int xsvf_fd;

/* map xsvf tap state to an openocd "tap_state_t" */
static tap_state_t xsvf_to_tap(int xsvf_state)
{
	tap_state_t ret;

	switch (xsvf_state) {
		case XSV_RESET:
			ret = TAP_RESET;
			break;
		case XSV_IDLE:
			ret = TAP_IDLE;
			break;
		case XSV_DRSELECT:
			ret = TAP_DRSELECT;
			break;
		case XSV_DRCAPTURE:
			ret = TAP_DRCAPTURE;
			break;
		case XSV_DRSHIFT:
			ret = TAP_DRSHIFT;
			break;
		case XSV_DREXIT1:
			ret = TAP_DREXIT1;
			break;
		case XSV_DRPAUSE:
			ret = TAP_DRPAUSE;
			break;
		case XSV_DREXIT2:
			ret = TAP_DREXIT2;
			break;
		case XSV_DRUPDATE:
			ret = TAP_DRUPDATE;
			break;
		case XSV_IRSELECT:
			ret = TAP_IRSELECT;
			break;
		case XSV_IRCAPTURE:
			ret = TAP_IRCAPTURE;
			break;
		case XSV_IRSHIFT:
			ret = TAP_IRSHIFT;
			break;
		case XSV_IREXIT1:
			ret = TAP_IREXIT1;
			break;
		case XSV_IRPAUSE:
			ret = TAP_IRPAUSE;
			break;
		case XSV_IREXIT2:
			ret = TAP_IREXIT2;
			break;
		case XSV_IRUPDATE:
			ret = TAP_IRUPDATE;
			break;
		default:
			LOG_ERROR("UNKNOWN XSVF STATE 0x%02X", xsvf_state);
			exit(1);
	}

	return ret;
}

static int xsvf_read_buffer(int num_bits, int fd, uint8_t *buf)
{
	int num_bytes;

	for (num_bytes = (num_bits + 7) / 8; num_bytes > 0; num_bytes--) {
		/* reverse the order of bytes as they are read sequentially from file */
		if (read(fd, buf + num_bytes - 1, 1) < 0)
			return ERROR_XSVF_EOF;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_xsvf_command)
{
	uint8_t *dr_out_buf = NULL;				/* from host to device (TDI) */
	uint8_t *dr_in_buf = NULL;				/* from device to host (TDO) */
	uint8_t *dr_in_mask = NULL;

	int xsdrsize = 0;
	int xruntest = 0;					/* number of TCK cycles OR *microseconds */
	int xrepeat = 0;					/* number of retries */

	tap_state_t xendir = TAP_IDLE;			/* see page 8 of the SVF spec, initial
							 *xendir to be TAP_IDLE */
	tap_state_t xenddr = TAP_IDLE;

	uint8_t opcode;
	uint8_t uc = 0;
	long file_offset = 0;

	int loop_count = 0;
	tap_state_t loop_state = TAP_IDLE;
	int loop_clocks = 0;
	int loop_usecs = 0;

	int do_abort = 0;
	int unsupported = 0;
	int tdo_mismatch = 0;
	int result;
	int verbose = 1;

	bool collecting_path = false;
	tap_state_t path[XSTATE_MAX_PATH];
	unsigned pathlen = 0;

	/* a flag telling whether to clock TCK during waits,
	 * or simply sleep, controlled by virt2
	 */
	int runtest_requires_tck = 0;

	/* use NULL to indicate a "plain" xsvf file which accounts for
	 * additional devices in the scan chain, otherwise the device
	 * that should be affected
	*/
	struct jtag_tap *tap = NULL;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* we mess with CMD_ARGV starting point below, snapshot filename here */
	const char *filename = CMD_ARGV[1];

	if (strcmp(CMD_ARGV[0], "plain") != 0) {
		tap = jtag_tap_by_string(CMD_ARGV[0]);
		if (!tap) {
			command_print(CMD, "Tap: %s unknown", CMD_ARGV[0]);
			return ERROR_FAIL;
		}
	}

	xsvf_fd = open(filename, O_RDONLY);
	if (xsvf_fd < 0) {
		command_print(CMD, "file \"%s\" not found", filename);
		return ERROR_FAIL;
	}

	/* if this argument is present, then interpret xruntest counts as TCK cycles rather than as
	 *usecs */
	if ((CMD_ARGC > 2) && (strcmp(CMD_ARGV[2], "virt2") == 0)) {
		runtest_requires_tck = 1;
		--CMD_ARGC;
		++CMD_ARGV;
	}

	if ((CMD_ARGC > 2) && (strcmp(CMD_ARGV[2], "quiet") == 0))
		verbose = 0;

	LOG_WARNING("XSVF support in OpenOCD is limited. Consider using SVF instead");
	LOG_USER("xsvf processing file: \"%s\"", filename);

	while (read(xsvf_fd, &opcode, 1) > 0) {
		/* record the position of this opcode within the file */
		file_offset = lseek(xsvf_fd, 0, SEEK_CUR) - 1;

		/* maybe collect another state for a pathmove();
		 * or terminate a path.
		 */
		if (collecting_path) {
			tap_state_t mystate;

			switch (opcode) {
				case XCOMMENT:
					/* ignore/show comments between XSTATE ops */
					break;
				case XSTATE:
					/* try to collect another transition */
					if (pathlen == XSTATE_MAX_PATH) {
						LOG_ERROR("XSVF: path too long");
						do_abort = 1;
						break;
					}

					if (read(xsvf_fd, &uc, 1) < 0) {
						do_abort = 1;
						break;
					}

					mystate = xsvf_to_tap(uc);
					path[pathlen++] = mystate;

					LOG_DEBUG("XSTATE 0x%02X %s", uc,
					tap_state_name(mystate));

					/* If path is incomplete, collect more */
					if (!svf_tap_state_is_stable(mystate))
						continue;

					/* Else execute the path transitions we've
					 * collected so far.
					 *
					 * NOTE:  Punting on the saved path is not
					 * strictly correct, but we must to do this
					 * unless jtag_add_pathmove() stops rejecting
					 * paths containing RESET.  This is probably
					 * harmless, since there aren't many options
					 * for going from a stable state to reset;
					 * at the worst, we may issue extra clocks
					 * once we get to RESET.
					 */
					if (mystate == TAP_RESET) {
						LOG_WARNING("XSVF: dodgey RESET");
						path[0] = mystate;
					}

				/* FALL THROUGH */
				default:
					/* Execute the path we collected
					 *
					 * NOTE: OpenOCD requires something that XSVF
					 * doesn't:  the last TAP state in the path
					 * must be stable.  In practice, tools that
					 * create XSVF seem to follow that rule too.
					 */
					collecting_path = false;

					if (path[0] == TAP_RESET)
						jtag_add_tlr();
					else
						jtag_add_pathmove(pathlen, path);

					result = jtag_execute_queue();
					if (result != ERROR_OK) {
						LOG_ERROR("XSVF: pathmove error %d", result);
						do_abort = 1;
						break;
					}
					continue;
			}
		}

		switch (opcode) {
			case XCOMPLETE:
				LOG_DEBUG("XCOMPLETE");

				result = jtag_execute_queue();
				if (result != ERROR_OK) {
					tdo_mismatch = 1;
					break;
				}
				break;

			case XTDOMASK:
				LOG_DEBUG("XTDOMASK");
				if (dr_in_mask &&
						(xsvf_read_buffer(xsdrsize, xsvf_fd, dr_in_mask) != ERROR_OK))
					do_abort = 1;
				break;

			case XRUNTEST:
			{
				uint8_t xruntest_buf[4];

				if (read(xsvf_fd, xruntest_buf, 4) < 0) {
					do_abort = 1;
					break;
				}

				xruntest = be_to_h_u32(xruntest_buf);
				LOG_DEBUG("XRUNTEST %d 0x%08X", xruntest, xruntest);
			}
			break;

			case XREPEAT:
			{
				uint8_t myrepeat;

				if (read(xsvf_fd, &myrepeat, 1) < 0)
					do_abort = 1;
				else {
					xrepeat = myrepeat;
					LOG_DEBUG("XREPEAT %d", xrepeat);
				}
			}
			break;

			case XSDRSIZE:
			{
				uint8_t xsdrsize_buf[4];

				if (read(xsvf_fd, xsdrsize_buf, 4) < 0) {
					do_abort = 1;
					break;
				}

				xsdrsize = be_to_h_u32(xsdrsize_buf);
				LOG_DEBUG("XSDRSIZE %d", xsdrsize);

				free(dr_out_buf);
				free(dr_in_buf);
				free(dr_in_mask);

				dr_out_buf = malloc((xsdrsize + 7) / 8);
				dr_in_buf = malloc((xsdrsize + 7) / 8);
				dr_in_mask = malloc((xsdrsize + 7) / 8);
			}
			break;

			case XSDR:		/* these two are identical except for the dr_in_buf */
			case XSDRTDO:
			{
				int limit = xrepeat;
				int matched = 0;
				int attempt;

				const char *op_name = (opcode == XSDR ? "XSDR" : "XSDRTDO");

				if (xsvf_read_buffer(xsdrsize, xsvf_fd, dr_out_buf) != ERROR_OK) {
					do_abort = 1;
					break;
				}

				if (opcode == XSDRTDO) {
					if (xsvf_read_buffer(xsdrsize, xsvf_fd,
						dr_in_buf)  != ERROR_OK) {
						do_abort = 1;
						break;
					}
				}

				if (limit < 1)
					limit = 1;

				LOG_DEBUG("%s %d", op_name, xsdrsize);

				for (attempt = 0; attempt < limit; ++attempt) {
					struct scan_field field;

					if (attempt > 0) {
						/* perform the XC9500 exception handling sequence shown in xapp067.pdf and
						 * illustrated in pseudo code at end of this file.  We start from state
						 * DRPAUSE:
						 * go to Exit2-DR
						 * go to Shift-DR
						 * go to Exit1-DR
						 * go to Update-DR
						 * go to Run-Test/Idle
						 *
						 * This sequence should be harmless for other devices, and it
						 * will be skipped entirely if xrepeat is set to zero.
						 */

						static tap_state_t exception_path[] = {
							TAP_DREXIT2,
							TAP_DRSHIFT,
							TAP_DREXIT1,
							TAP_DRUPDATE,
							TAP_IDLE,
						};

						jtag_add_pathmove(ARRAY_SIZE(exception_path), exception_path);

						if (verbose)
							LOG_USER("%s mismatch, xsdrsize=%d retry=%d",
									op_name,
									xsdrsize,
									attempt);
					}

					field.num_bits = xsdrsize;
					field.out_value = dr_out_buf;
					field.in_value = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);

					if (tap == NULL)
						jtag_add_plain_dr_scan(field.num_bits,
								field.out_value,
								field.in_value,
								TAP_DRPAUSE);
					else
						jtag_add_dr_scan(tap, 1, &field, TAP_DRPAUSE);

					jtag_check_value_mask(&field, dr_in_buf, dr_in_mask);

					free(field.in_value);

					/* LOG_DEBUG("FLUSHING QUEUE"); */
					result = jtag_execute_queue();
					if (result == ERROR_OK) {
						matched = 1;
						break;
					}
				}

				if (!matched) {
					LOG_USER("%s mismatch", op_name);
					tdo_mismatch = 1;
					break;
				}

				/* See page 19 of XSVF spec regarding opcode "XSDR" */
				if (xruntest) {
					result = svf_add_statemove(TAP_IDLE);
					if (result != ERROR_OK)
						return result;

					if (runtest_requires_tck)
						jtag_add_clocks(xruntest);
					else
						jtag_add_sleep(xruntest);
				} else if (xendir != TAP_DRPAUSE) {
					/* we are already in TAP_DRPAUSE */
					result = svf_add_statemove(xenddr);
					if (result != ERROR_OK)
						return result;
				}
			}
			break;

			case XSETSDRMASKS:
				LOG_ERROR("unsupported XSETSDRMASKS");
				unsupported = 1;
				break;

			case XSDRINC:
				LOG_ERROR("unsupported XSDRINC");
				unsupported = 1;
				break;

			case XSDRB:
				LOG_ERROR("unsupported XSDRB");
				unsupported = 1;
				break;

			case XSDRC:
				LOG_ERROR("unsupported XSDRC");
				unsupported = 1;
				break;

			case XSDRE:
				LOG_ERROR("unsupported XSDRE");
				unsupported = 1;
				break;

			case XSDRTDOB:
				LOG_ERROR("unsupported XSDRTDOB");
				unsupported = 1;
				break;

			case XSDRTDOC:
				LOG_ERROR("unsupported XSDRTDOC");
				unsupported = 1;
				break;

			case XSDRTDOE:
				LOG_ERROR("unsupported XSDRTDOE");
				unsupported = 1;
				break;

			case XSTATE:
			{
				tap_state_t mystate;

				if (read(xsvf_fd, &uc, 1) < 0) {
					do_abort = 1;
					break;
				}

				mystate = xsvf_to_tap(uc);

				LOG_DEBUG("XSTATE 0x%02X %s", uc, tap_state_name(mystate));

				if (mystate == TAP_INVALID) {
					LOG_ERROR("XSVF: bad XSTATE %02x", uc);
					do_abort = 1;
					break;
				}

				/* NOTE: the current state is SVF-stable! */

				/* no change == NOP */
				if (mystate == cmd_queue_cur_state
						&& mystate != TAP_RESET)
					break;

				/* Hand off to SVF? */
				if (svf_tap_state_is_stable(mystate)) {
					result = svf_add_statemove(mystate);
					if (result != ERROR_OK)
						unsupported = 1;
					break;
				}

				/*
				 * A sequence of XSTATE transitions, each TAP
				 * state adjacent to the previous one.  Start
				 * collecting them.
				 */
				collecting_path = true;
				pathlen = 1;
				path[0] = mystate;
			}
			break;

			case XENDIR:

				if (read(xsvf_fd, &uc, 1) < 0) {
					do_abort = 1;
					break;
				}

				/* see page 22 of XSVF spec */
				if (uc == 0)
					xendir = TAP_IDLE;
				else if (uc == 1)
					xendir = TAP_IRPAUSE;
				else {
					LOG_ERROR("illegial XENDIR argument: 0x%02X", uc);
					unsupported = 1;
					break;
				}

				LOG_DEBUG("XENDIR 0x%02X %s", uc, tap_state_name(xendir));
				break;

			case XENDDR:

				if (read(xsvf_fd, &uc, 1) < 0) {
					do_abort = 1;
					break;
				}

				/* see page 22 of XSVF spec */
				if (uc == 0)
					xenddr = TAP_IDLE;
				else if (uc == 1)
					xenddr = TAP_DRPAUSE;
				else {
					LOG_ERROR("illegial XENDDR argument: 0x%02X", uc);
					unsupported = 1;
					break;
				}

				LOG_DEBUG("XENDDR %02X %s", uc, tap_state_name(xenddr));
				break;

			case XSIR:
			case XSIR2:
			{
				uint8_t short_buf[2];
				uint8_t *ir_buf;
				int bitcount;
				tap_state_t my_end_state = xruntest ? TAP_IDLE : xendir;

				if (opcode == XSIR) {
					/* one byte bitcount */
					if (read(xsvf_fd, short_buf, 1) < 0) {
						do_abort = 1;
						break;
					}
					bitcount = short_buf[0];
					LOG_DEBUG("XSIR %d", bitcount);
				} else {
					if (read(xsvf_fd, short_buf, 2) < 0) {
						do_abort = 1;
						break;
					}
					bitcount = be_to_h_u16(short_buf);
					LOG_DEBUG("XSIR2 %d", bitcount);
				}

				ir_buf = malloc((bitcount + 7) / 8);

				if (xsvf_read_buffer(bitcount, xsvf_fd, ir_buf) != ERROR_OK)
					do_abort = 1;
				else {
					struct scan_field field;

					field.num_bits = bitcount;
					field.out_value = ir_buf;

					field.in_value = NULL;

					if (tap == NULL)
						jtag_add_plain_ir_scan(field.num_bits,
								field.out_value, field.in_value, my_end_state);
					else
						jtag_add_ir_scan(tap, &field, my_end_state);

					if (xruntest) {
						if (runtest_requires_tck)
							jtag_add_clocks(xruntest);
						else
							jtag_add_sleep(xruntest);
					}

					/* Note that an -irmask of non-zero in your config file
					 * can cause this to fail.  Setting -irmask to zero cand work
					 * around the problem.
					 */

					/* LOG_DEBUG("FLUSHING QUEUE"); */
					result = jtag_execute_queue();
					if (result != ERROR_OK)
						tdo_mismatch = 1;
				}
				free(ir_buf);
			}
			break;

			case XCOMMENT:
			{
				unsigned int ndx = 0;
				char comment[128];

				do {
					if (read(xsvf_fd, &uc, 1) < 0) {
						do_abort = 1;
						break;
					}

					if (ndx < sizeof(comment)-1)
						comment[ndx++] = uc;

				} while (uc != 0);

				comment[sizeof(comment)-1] = 0;		/* regardless, terminate */
				if (verbose)
					LOG_USER("# %s", comment);
			}
			break;

			case XWAIT:
			{
				/* expected in stream:
				   XWAIT <uint8_t wait_state> <uint8_t end_state> <uint32_t usecs>
				*/

				uint8_t wait_local;
				uint8_t end;
				uint8_t delay_buf[4];

				tap_state_t wait_state;
				tap_state_t end_state;
				int delay;

				if (read(xsvf_fd, &wait_local, 1) < 0
					|| read(xsvf_fd, &end, 1) < 0
					|| read(xsvf_fd, delay_buf, 4) < 0) {
						do_abort = 1;
						break;
				}

				wait_state = xsvf_to_tap(wait_local);
				end_state  = xsvf_to_tap(end);
				delay = be_to_h_u32(delay_buf);

				LOG_DEBUG("XWAIT %s %s usecs:%d", tap_state_name(
						wait_state), tap_state_name(end_state), delay);

				if (runtest_requires_tck && wait_state == TAP_IDLE)
					jtag_add_runtest(delay, end_state);
				else {
					/* FIXME handle statemove errors ... */
					result = svf_add_statemove(wait_state);
					if (result != ERROR_OK)
						return result;
					jtag_add_sleep(delay);
					result = svf_add_statemove(end_state);
					if (result != ERROR_OK)
						return result;
				}
			}
			break;

			case XWAITSTATE:
			{
				/* expected in stream:
				 * XWAITSTATE <uint8_t wait_state> <uint8_t end_state> <uint32_t clock_count>
				 * <uint32_t usecs>
				*/

				uint8_t clock_buf[4];
				uint8_t usecs_buf[4];
				uint8_t wait_local;
				uint8_t end;
				tap_state_t wait_state;
				tap_state_t end_state;
				int clock_count;
				int usecs;

				if (read(xsvf_fd, &wait_local, 1) < 0
						||  read(xsvf_fd, &end, 1) < 0
						||  read(xsvf_fd, clock_buf, 4) < 0
						||  read(xsvf_fd, usecs_buf, 4) < 0) {
					do_abort = 1;
					break;
				}

				wait_state = xsvf_to_tap(wait_local);
				end_state  = xsvf_to_tap(end);

				clock_count = be_to_h_u32(clock_buf);
				usecs = be_to_h_u32(usecs_buf);

				LOG_DEBUG("XWAITSTATE %s %s clocks:%i usecs:%i",
						tap_state_name(wait_state),
						tap_state_name(end_state),
						clock_count, usecs);

				/* the following states are 'stable', meaning that they have a transition
				 * in the state diagram back to themselves.  This is necessary because we will
				 * be issuing a number of clocks in this state.  This set of allowed states is also
				 * determined by the SVF RUNTEST command's allowed states.
				 */
				if (!svf_tap_state_is_stable(wait_state)) {
					LOG_ERROR("illegal XWAITSTATE wait_state: \"%s\"",
							tap_state_name(wait_state));
					unsupported = 1;
					/* REVISIT "break" so we won't run? */
				}

				/* FIXME handle statemove errors ... */
				result = svf_add_statemove(wait_state);
				if (result != ERROR_OK)
					return result;

				jtag_add_clocks(clock_count);
				jtag_add_sleep(usecs);

				result = svf_add_statemove(end_state);
				if (result != ERROR_OK)
					return result;
			}
			break;

			case LCOUNT:
			{
				/* expected in stream:
				 * LCOUNT <uint32_t loop_count>
				*/
				uint8_t count_buf[4];

				if (read(xsvf_fd, count_buf, 4) < 0) {
					do_abort = 1;
					break;
				}

				loop_count = be_to_h_u32(count_buf);
				LOG_DEBUG("LCOUNT %d", loop_count);
			}
			break;

			case LDELAY:
			{
				/* expected in stream:
				 * LDELAY <uint8_t wait_state> <uint32_t clock_count> <uint32_t usecs_to_sleep>
				*/
				uint8_t state;
				uint8_t clock_buf[4];
				uint8_t usecs_buf[4];

				if (read(xsvf_fd, &state, 1) < 0
						|| read(xsvf_fd, clock_buf, 4) < 0
						|| read(xsvf_fd, usecs_buf, 4) < 0) {
					do_abort = 1;
					break;
				}

				/* NOTE:  loop_state must be stable! */
				loop_state  = xsvf_to_tap(state);
				loop_clocks = be_to_h_u32(clock_buf);
				loop_usecs  = be_to_h_u32(usecs_buf);

				LOG_DEBUG("LDELAY %s clocks:%d usecs:%d", tap_state_name(
						loop_state), loop_clocks, loop_usecs);
			}
			break;

			/* LSDR is more like XSDRTDO than it is like XSDR.  It uses LDELAY which
			 * comes with clocks !AND! sleep requirements.
			 */
			case LSDR:
			{
				int limit = loop_count;
				int matched = 0;
				int attempt;

				LOG_DEBUG("LSDR");

				if (xsvf_read_buffer(xsdrsize, xsvf_fd, dr_out_buf) != ERROR_OK
						|| xsvf_read_buffer(xsdrsize, xsvf_fd, dr_in_buf) != ERROR_OK) {
					do_abort = 1;
					break;
				}

				if (limit < 1)
					limit = 1;

				for (attempt = 0; attempt < limit; ++attempt) {
					struct scan_field field;

					result = svf_add_statemove(loop_state);
					if (result != ERROR_OK) {
						free(dr_in_mask);
						return result;
					}
					jtag_add_clocks(loop_clocks);
					jtag_add_sleep(loop_usecs);

					field.num_bits = xsdrsize;
					field.out_value = dr_out_buf;
					field.in_value = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);

					if (attempt > 0 && verbose)
						LOG_USER("LSDR retry %d", attempt);

					if (tap == NULL)
						jtag_add_plain_dr_scan(field.num_bits,
								field.out_value,
								field.in_value,
								TAP_DRPAUSE);
					else
						jtag_add_dr_scan(tap, 1, &field, TAP_DRPAUSE);

					jtag_check_value_mask(&field, dr_in_buf, dr_in_mask);

					free(field.in_value);


					/* LOG_DEBUG("FLUSHING QUEUE"); */
					result = jtag_execute_queue();
					if (result == ERROR_OK) {
						matched = 1;
						break;
					}
				}

				if (!matched) {
					LOG_USER("LSDR mismatch");
					tdo_mismatch = 1;
					break;
				}
			}
			break;

			case XTRST:
			{
				uint8_t trst_mode;

				if (read(xsvf_fd, &trst_mode, 1) < 0) {
					do_abort = 1;
					break;
				}

				switch (trst_mode) {
				case XTRST_ON:
					jtag_add_reset(1, 0);
					break;
				case XTRST_OFF:
				case XTRST_Z:
					jtag_add_reset(0, 0);
					break;
				case XTRST_ABSENT:
					break;
				default:
					LOG_ERROR("XTRST mode argument (0x%02X) out of range", trst_mode);
					do_abort = 1;
				}
			}
			break;

			default:
				LOG_ERROR("unknown xsvf command (0x%02X)", uc);
				unsupported = 1;
		}

		if (do_abort || unsupported || tdo_mismatch) {
			LOG_DEBUG("xsvf failed, setting taps to reasonable state");

			/* upon error, return the TAPs to a reasonable state */
			result = svf_add_statemove(TAP_IDLE);
			if (result != ERROR_OK)
				return result;
			result = jtag_execute_queue();
			if (result != ERROR_OK)
				return result;
			break;
		}
	}

	if (tdo_mismatch) {
		command_print(CMD,
			"TDO mismatch, somewhere near offset %lu in xsvf file, aborting",
			file_offset);

		return ERROR_FAIL;
	}

	if (unsupported) {
		off_t offset = lseek(xsvf_fd, 0, SEEK_CUR) - 1;
		command_print(CMD,
			"unsupported xsvf command (0x%02X) at offset %jd, aborting",
			uc, (intmax_t)offset);
		return ERROR_FAIL;
	}

	if (do_abort) {
		command_print(CMD, "premature end of xsvf file detected, aborting");
		return ERROR_FAIL;
	}

	free(dr_out_buf);
	free(dr_in_buf);
	free(dr_in_mask);

	close(xsvf_fd);

	command_print(CMD, "XSVF file programmed successfully");

	return ERROR_OK;
}

static const struct command_registration xsvf_command_handlers[] = {
	{
		.name = "xsvf",
		.handler = handle_xsvf_command,
		.mode = COMMAND_EXEC,
		.help = "Runs a XSVF file.  If 'virt2' is given, xruntest "
			"counts are interpreted as TCK cycles rather than "
			"as microseconds.  Without the 'quiet' option, all "
			"comments, retries, and mismatches will be reported.",
		.usage = "(tapname|'plain') filename ['virt2'] ['quiet']",
	},
	COMMAND_REGISTRATION_DONE
};

int xsvf_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, xsvf_command_handlers);
}

/*

PSUEDO-Code from Xilinx Appnote XAPP067.pdf :

the following pseudo code clarifies the intent of the xrepeat support.The
flow given is for the entire processing of an SVF file, not an XSVF file.
No idea if this is just for the XC9500/XL/XV devices or all Xilinx parts.

"Pseudo-Code Algorithm for SVF-Based ISP"

1. Go to Test-Logic-Reset state
2. Go to Run-Test Idle state
3. Read SVF record

4. if SIR record then
go to Shift-IR state
Scan in <TDI value>

5. else if SDR record then
set <repeat count> to 0
store <TDI value> as <current TDI value>
store <TDO value> as <current TDO value>
6. go to Shift-DR state
scan in <current TDI value>
if < current TDO value > is specified then
if < current TDO value > does not equal <actual TDO value> then
if < repeat count > > 32 then
LOG ERROR
go to Run-Test Idle state
go to Step 3
end if
go to Pause-DR
go to Exit2-DR
go to Shift-DR
go to Exit1-DR
go to Update-DR
go to Run-Test/Idle
increment <repeat count> by 1
pause <current pause time> microseconds
go to Step 6)
end if
else
	go to Run-Test Idle state
	go to Step 3
	endif
	else if RUNTEST record then
	pause tester for < TCK value > microseconds
	store <TCK value> as <current pause time>
	end if

*/
