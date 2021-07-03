/***************************************************************************
 *    Copyright (C) 2009 by Simon Qian                                     *
 *    SimonQian@SimonQian.com                                              *
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

#include <jtag/jtag.h>
#include "svf.h"
#include "helper/system.h"
#include <helper/time_support.h>

/* SVF command */
enum svf_command {
	ENDDR,
	ENDIR,
	FREQUENCY,
	HDR,
	HIR,
	PIO,
	PIOMAP,
	RUNTEST,
	SDR,
	SIR,
	STATE,
	TDR,
	TIR,
	TRST,
};

static const char *svf_command_name[14] = {
	"ENDDR",
	"ENDIR",
	"FREQUENCY",
	"HDR",
	"HIR",
	"PIO",
	"PIOMAP",
	"RUNTEST",
	"SDR",
	"SIR",
	"STATE",
	"TDR",
	"TIR",
	"TRST"
};

enum trst_mode {
	TRST_ON,
	TRST_OFF,
	TRST_Z,
	TRST_ABSENT
};

static const char *svf_trst_mode_name[4] = {
	"ON",
	"OFF",
	"Z",
	"ABSENT"
};

struct svf_statemove {
	tap_state_t from;
	tap_state_t to;
	uint32_t num_of_moves;
	tap_state_t paths[8];
};

/*
 * These paths are from the SVF specification for the STATE command, to be
 * used when the STATE command only includes the final state.  The first
 * element of the path is the "from" (current) state, and the last one is
 * the "to" (target) state.
 *
 * All specified paths are the shortest ones in the JTAG spec, and are thus
 * not (!!) exact matches for the paths used elsewhere in OpenOCD.  Note
 * that PAUSE-to-PAUSE transitions all go through UPDATE and then CAPTURE,
 * which has specific effects on the various registers; they are not NOPs.
 *
 * Paths to RESET are disabled here.  As elsewhere in OpenOCD, and in XSVF
 * and many SVF implementations, we don't want to risk missing that state.
 * To get to RESET, always we ignore the current state.
 */
static const struct svf_statemove svf_statemoves[] = {
	/* from			to				num_of_moves,	paths[8] */
/*	{TAP_RESET,		TAP_RESET,		1,				{TAP_RESET}}, */
	{TAP_RESET,		TAP_IDLE,		2,				{TAP_RESET, TAP_IDLE} },
	{TAP_RESET,		TAP_DRPAUSE,	6,				{TAP_RESET, TAP_IDLE, TAP_DRSELECT,
														TAP_DRCAPTURE, TAP_DREXIT1, TAP_DRPAUSE} },
	{TAP_RESET,		TAP_IRPAUSE,	7,				{TAP_RESET, TAP_IDLE, TAP_DRSELECT,
														TAP_IRSELECT, TAP_IRCAPTURE,
														TAP_IREXIT1, TAP_IRPAUSE} },

/*	{TAP_IDLE,		TAP_RESET,		4,				{TAP_IDLE,
 * TAP_DRSELECT, TAP_IRSELECT, TAP_RESET}}, */
	{TAP_IDLE,		TAP_IDLE,		1,				{TAP_IDLE} },
	{TAP_IDLE,		TAP_DRPAUSE,	5,				{TAP_IDLE, TAP_DRSELECT, TAP_DRCAPTURE,
														TAP_DREXIT1, TAP_DRPAUSE} },
	{TAP_IDLE,		TAP_IRPAUSE,	6,				{TAP_IDLE, TAP_DRSELECT, TAP_IRSELECT,
														TAP_IRCAPTURE, TAP_IREXIT1, TAP_IRPAUSE} },

/*	{TAP_DRPAUSE,	TAP_RESET,		6,				{TAP_DRPAUSE,
 * TAP_DREXIT2, TAP_DRUPDATE, TAP_DRSELECT, TAP_IRSELECT, TAP_RESET}}, */
	{TAP_DRPAUSE,	TAP_IDLE,		4,				{TAP_DRPAUSE, TAP_DREXIT2, TAP_DRUPDATE,
														TAP_IDLE} },
	{TAP_DRPAUSE,	TAP_DRPAUSE,	7,				{TAP_DRPAUSE, TAP_DREXIT2, TAP_DRUPDATE,
														TAP_DRSELECT, TAP_DRCAPTURE,
														TAP_DREXIT1, TAP_DRPAUSE} },
	{TAP_DRPAUSE,	TAP_IRPAUSE,	8,				{TAP_DRPAUSE, TAP_DREXIT2, TAP_DRUPDATE,
														TAP_DRSELECT, TAP_IRSELECT,
														TAP_IRCAPTURE, TAP_IREXIT1, TAP_IRPAUSE} },

/*	{TAP_IRPAUSE,	TAP_RESET,		6,				{TAP_IRPAUSE,
 * TAP_IREXIT2, TAP_IRUPDATE, TAP_DRSELECT, TAP_IRSELECT, TAP_RESET}}, */
	{TAP_IRPAUSE,	TAP_IDLE,		4,				{TAP_IRPAUSE, TAP_IREXIT2, TAP_IRUPDATE,
														TAP_IDLE} },
	{TAP_IRPAUSE,	TAP_DRPAUSE,	7,				{TAP_IRPAUSE, TAP_IREXIT2, TAP_IRUPDATE,
														TAP_DRSELECT, TAP_DRCAPTURE,
														TAP_DREXIT1, TAP_DRPAUSE} },
	{TAP_IRPAUSE,	TAP_IRPAUSE,	8,				{TAP_IRPAUSE, TAP_IREXIT2, TAP_IRUPDATE,
														TAP_DRSELECT, TAP_IRSELECT,
														TAP_IRCAPTURE, TAP_IREXIT1, TAP_IRPAUSE} }
};

#define XXR_TDI				(1 << 0)
#define XXR_TDO				(1 << 1)
#define XXR_MASK			(1 << 2)
#define XXR_SMASK			(1 << 3)
struct svf_xxr_para {
	int len;
	int data_mask;
	uint8_t *tdi;
	uint8_t *tdo;
	uint8_t *mask;
	uint8_t *smask;
};

struct svf_para {
	float frequency;
	tap_state_t ir_end_state;
	tap_state_t dr_end_state;
	tap_state_t runtest_run_state;
	tap_state_t runtest_end_state;
	enum trst_mode trst_mode;

	struct svf_xxr_para hir_para;
	struct svf_xxr_para hdr_para;
	struct svf_xxr_para tir_para;
	struct svf_xxr_para tdr_para;
	struct svf_xxr_para sir_para;
	struct svf_xxr_para sdr_para;
};

static struct svf_para svf_para;
static const struct svf_para svf_para_init = {
/*	frequency, ir_end_state, dr_end_state, runtest_run_state, runtest_end_state, trst_mode */
	0,			TAP_IDLE,		TAP_IDLE,	TAP_IDLE,		TAP_IDLE,		TRST_Z,
/*	hir_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	hdr_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	tir_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	tdr_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	sir_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
/*	sdr_para */
/*	{len,	data_mask,	tdi,	tdo,	mask,	smask}, */
	{0,			0,		NULL,	NULL,	NULL,	NULL},
};

struct svf_check_tdo_para {
	int line_num;		/* used to record line number of the check operation */
	/* so more information could be printed */
	int enabled;		/* check is enabled or not */
	int buffer_offset;	/* buffer_offset to buffers */
	int bit_len;		/* bit length to check */
};

#define SVF_CHECK_TDO_PARA_SIZE 1024
static struct svf_check_tdo_para *svf_check_tdo_para;
static int svf_check_tdo_para_index;

static int svf_read_command_from_file(FILE *fd);
static int svf_check_tdo(void);
static int svf_add_check_para(uint8_t enabled, int buffer_offset, int bit_len);
static int svf_run_command(struct command_context *cmd_ctx, char *cmd_str);
static int svf_execute_tap(void);

static FILE *svf_fd;
static char *svf_read_line;
static size_t svf_read_line_size;
static char *svf_command_buffer;
static size_t svf_command_buffer_size;
static int svf_line_number;
static int svf_getline(char **lineptr, size_t *n, FILE *stream);

#define SVF_MAX_BUFFER_SIZE_TO_COMMIT   (1024 * 1024)
static uint8_t *svf_tdi_buffer, *svf_tdo_buffer, *svf_mask_buffer;
static int svf_buffer_index, svf_buffer_size;
static int svf_quiet;
static int svf_nil;
static int svf_ignore_error;

/* Targeting particular tap */
static int svf_tap_is_specified;
static int svf_set_padding(struct svf_xxr_para *para, int len, unsigned char tdi);

/* Progress Indicator */
static int svf_progress_enabled;
static long svf_total_lines;
static int svf_percentage;
static int svf_last_printed_percentage = -1;

/*
 * macro is used to print the svf hex buffer at desired debug level
 * DEBUG, INFO, ERROR, USER
 */
#define SVF_BUF_LOG(_lvl, _buf, _nbits, _desc)							\
	svf_hexbuf_print(LOG_LVL_##_lvl,  __FILE__, __LINE__, __func__, _buf, _nbits, _desc)

static void svf_hexbuf_print(int dbg_lvl, const char *file, unsigned line,
							 const char *function, const uint8_t *buf,
							 int bit_len, const char *desc)
{
	int j, len = 0;
	int byte_len = DIV_ROUND_UP(bit_len, 8);
	int msbits = bit_len % 8;

	/* allocate 2 bytes per hex digit */
	char *prbuf = malloc((byte_len * 2) + 2 + 1);
	if (!prbuf)
		return;

	/* print correct number of bytes, mask excess bits where applicable */
	uint8_t msb = buf[byte_len - 1] & (msbits ? (1 << msbits) - 1 : 0xff);
	len = sprintf(prbuf, msbits <= 4 ? "0x%01"PRIx8 : "0x%02"PRIx8, msb);
	for (j = byte_len - 2; j >= 0; j--)
		len += sprintf(prbuf + len, "%02"PRIx8, buf[j]);

	log_printf_lf(dbg_lvl, file, line, function, "%8s = %s", desc ? desc : " ", prbuf);

	free(prbuf);
}

static int svf_realloc_buffers(size_t len)
{
	void *ptr;

	if (svf_execute_tap() != ERROR_OK)
		return ERROR_FAIL;

	ptr = realloc(svf_tdi_buffer, len);
	if (!ptr)
		return ERROR_FAIL;
	svf_tdi_buffer = ptr;

	ptr = realloc(svf_tdo_buffer, len);
	if (!ptr)
		return ERROR_FAIL;
	svf_tdo_buffer = ptr;

	ptr = realloc(svf_mask_buffer, len);
	if (!ptr)
		return ERROR_FAIL;
	svf_mask_buffer = ptr;

	svf_buffer_size = len;

	return ERROR_OK;
}

static void svf_free_xxd_para(struct svf_xxr_para *para)
{
	if (para) {
		free(para->tdi);
		para->tdi = NULL;

		free(para->tdo);
		para->tdo = NULL;

		free(para->mask);
		para->mask = NULL;

		free(para->smask);
		para->smask = NULL;
	}
}

int svf_add_statemove(tap_state_t state_to)
{
	tap_state_t state_from = cmd_queue_cur_state;
	unsigned index_var;

	/* when resetting, be paranoid and ignore current state */
	if (state_to == TAP_RESET) {
		if (svf_nil)
			return ERROR_OK;

		jtag_add_tlr();
		return ERROR_OK;
	}

	for (index_var = 0; index_var < ARRAY_SIZE(svf_statemoves); index_var++) {
		if ((svf_statemoves[index_var].from == state_from)
				&& (svf_statemoves[index_var].to == state_to)) {
			if (svf_nil)
				continue;
						/* recorded path includes current state ... avoid
						 *extra TCKs! */
			if (svf_statemoves[index_var].num_of_moves > 1)
				jtag_add_pathmove(svf_statemoves[index_var].num_of_moves - 1,
					svf_statemoves[index_var].paths + 1);
			else
				jtag_add_pathmove(svf_statemoves[index_var].num_of_moves,
					svf_statemoves[index_var].paths);
			return ERROR_OK;
		}
	}
	LOG_ERROR("SVF: can not move to %s", tap_state_name(state_to));
	return ERROR_FAIL;
}

COMMAND_HANDLER(handle_svf_command)
{
#define SVF_MIN_NUM_OF_OPTIONS 1
#define SVF_MAX_NUM_OF_OPTIONS 5
	int command_num = 0;
	int ret = ERROR_OK;
	int64_t time_measure_ms;
	int time_measure_s, time_measure_m;

	/* use NULL to indicate a "plain" svf file which accounts for
	 * any additional devices in the scan chain, otherwise the device
	 * that should be affected
	*/
	struct jtag_tap *tap = NULL;

	if ((CMD_ARGC < SVF_MIN_NUM_OF_OPTIONS) || (CMD_ARGC > SVF_MAX_NUM_OF_OPTIONS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* parse command line */
	svf_quiet = 0;
	svf_nil = 0;
	svf_progress_enabled = 0;
	svf_ignore_error = 0;
	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		if (strcmp(CMD_ARGV[i], "-tap") == 0) {
			tap = jtag_tap_by_string(CMD_ARGV[i+1]);
			if (!tap) {
				command_print(CMD, "Tap: %s unknown", CMD_ARGV[i+1]);
				return ERROR_FAIL;
			}
			i++;
		} else if ((strcmp(CMD_ARGV[i],
				"quiet") == 0) || (strcmp(CMD_ARGV[i], "-quiet") == 0))
			svf_quiet = 1;
		else if ((strcmp(CMD_ARGV[i], "nil") == 0) || (strcmp(CMD_ARGV[i], "-nil") == 0))
			svf_nil = 1;
		else if ((strcmp(CMD_ARGV[i],
				  "progress") == 0) || (strcmp(CMD_ARGV[i], "-progress") == 0))
			svf_progress_enabled = 1;
		else if ((strcmp(CMD_ARGV[i],
				  "ignore_error") == 0) || (strcmp(CMD_ARGV[i], "-ignore_error") == 0))
			svf_ignore_error = 1;
		else {
			svf_fd = fopen(CMD_ARGV[i], "r");
			if (!svf_fd) {
				int err = errno;
				command_print(CMD, "open(\"%s\"): %s", CMD_ARGV[i], strerror(err));
				/* no need to free anything now */
				return ERROR_COMMAND_SYNTAX_ERROR;
			} else
				LOG_USER("svf processing file: \"%s\"", CMD_ARGV[i]);
		}
	}

	if (!svf_fd)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* get time */
	time_measure_ms = timeval_ms();

	/* init */
	svf_line_number = 0;
	svf_command_buffer_size = 0;

	svf_check_tdo_para_index = 0;
	svf_check_tdo_para = malloc(sizeof(struct svf_check_tdo_para) * SVF_CHECK_TDO_PARA_SIZE);
	if (!svf_check_tdo_para) {
		LOG_ERROR("not enough memory");
		ret = ERROR_FAIL;
		goto free_all;
	}

	svf_buffer_index = 0;
	/* double the buffer size */
	/* in case current command cannot be committed, and next command is a bit scan command */
	/* here is 32K bits for this big scan command, it should be enough */
	/* buffer will be reallocated if buffer size is not enough */
	if (svf_realloc_buffers(2 * SVF_MAX_BUFFER_SIZE_TO_COMMIT) != ERROR_OK) {
		ret = ERROR_FAIL;
		goto free_all;
	}

	memcpy(&svf_para, &svf_para_init, sizeof(svf_para));

	if (!svf_nil) {
		/* TAP_RESET */
		jtag_add_tlr();
	}

	if (tap) {
		/* Tap is specified, set header/trailer paddings */
		int header_ir_len = 0, header_dr_len = 0, trailer_ir_len = 0, trailer_dr_len = 0;
		struct jtag_tap *check_tap;

		svf_tap_is_specified = 1;

		for (check_tap = jtag_all_taps(); check_tap; check_tap = check_tap->next_tap) {
			if (check_tap->abs_chain_position < tap->abs_chain_position) {
				/* Header */
				header_ir_len += check_tap->ir_length;
				header_dr_len++;
			} else if (check_tap->abs_chain_position > tap->abs_chain_position) {
				/* Trailer */
				trailer_ir_len += check_tap->ir_length;
				trailer_dr_len++;
			}
		}

		/* HDR %d TDI (0) */
		if (svf_set_padding(&svf_para.hdr_para, header_dr_len, 0) != ERROR_OK) {
			LOG_ERROR("failed to set data header");
			return ERROR_FAIL;
		}

		/* HIR %d TDI (0xFF) */
		if (svf_set_padding(&svf_para.hir_para, header_ir_len, 0xFF) != ERROR_OK) {
			LOG_ERROR("failed to set instruction header");
			return ERROR_FAIL;
		}

		/* TDR %d TDI (0) */
		if (svf_set_padding(&svf_para.tdr_para, trailer_dr_len, 0) != ERROR_OK) {
			LOG_ERROR("failed to set data trailer");
			return ERROR_FAIL;
		}

		/* TIR %d TDI (0xFF) */
		if (svf_set_padding(&svf_para.tir_para, trailer_ir_len, 0xFF) != ERROR_OK) {
			LOG_ERROR("failed to set instruction trailer");
			return ERROR_FAIL;
		}
	}

	if (svf_progress_enabled) {
		/* Count total lines in file. */
		while (!feof(svf_fd)) {
			svf_getline(&svf_command_buffer, &svf_command_buffer_size, svf_fd);
			svf_total_lines++;
		}
		rewind(svf_fd);
	}
	while (svf_read_command_from_file(svf_fd) == ERROR_OK) {
		/* Log Output */
		if (svf_quiet) {
			if (svf_progress_enabled) {
				svf_percentage = ((svf_line_number * 20) / svf_total_lines) * 5;
				if (svf_last_printed_percentage != svf_percentage) {
					LOG_USER_N("\r%d%%    ", svf_percentage);
					svf_last_printed_percentage = svf_percentage;
				}
			}
		} else {
			if (svf_progress_enabled) {
				svf_percentage = ((svf_line_number * 20) / svf_total_lines) * 5;
				LOG_USER_N("%3d%%  %s", svf_percentage, svf_read_line);
			} else
				LOG_USER_N("%s", svf_read_line);
		}
		/* Run Command */
		if (svf_run_command(CMD_CTX, svf_command_buffer) != ERROR_OK) {
			LOG_ERROR("fail to run command at line %d", svf_line_number);
			ret = ERROR_FAIL;
			break;
		}
		command_num++;
	}

	if ((!svf_nil) && (jtag_execute_queue() != ERROR_OK))
		ret = ERROR_FAIL;
	else if (svf_check_tdo() != ERROR_OK)
		ret = ERROR_FAIL;

	/* print time */
	time_measure_ms = timeval_ms() - time_measure_ms;
	time_measure_s = time_measure_ms / 1000;
	time_measure_ms %= 1000;
	time_measure_m = time_measure_s / 60;
	time_measure_s %= 60;
	if (time_measure_ms < 1000)
		command_print(CMD,
			"\r\nTime used: %dm%ds%" PRId64 "ms ",
			time_measure_m,
			time_measure_s,
			time_measure_ms);

free_all:

	fclose(svf_fd);
	svf_fd = 0;

	/* free buffers */
	free(svf_command_buffer);
	svf_command_buffer = NULL;
	svf_command_buffer_size = 0;

	free(svf_check_tdo_para);
	svf_check_tdo_para = NULL;
	svf_check_tdo_para_index = 0;

	free(svf_tdi_buffer);
	svf_tdi_buffer = NULL;

	free(svf_tdo_buffer);
	svf_tdo_buffer = NULL;

	free(svf_mask_buffer);
	svf_mask_buffer = NULL;

	svf_buffer_index = 0;
	svf_buffer_size = 0;

	svf_free_xxd_para(&svf_para.hdr_para);
	svf_free_xxd_para(&svf_para.hir_para);
	svf_free_xxd_para(&svf_para.tdr_para);
	svf_free_xxd_para(&svf_para.tir_para);
	svf_free_xxd_para(&svf_para.sdr_para);
	svf_free_xxd_para(&svf_para.sir_para);

	if (ret == ERROR_OK)
		command_print(CMD,
			      "svf file programmed %s for %d commands with %d errors",
			      (svf_ignore_error > 1) ? "unsuccessfully" : "successfully",
			      command_num,
			      (svf_ignore_error > 1) ? (svf_ignore_error - 1) : 0);
	else
		command_print(CMD, "svf file programmed failed");

	svf_ignore_error = 0;
	return ret;
}

static int svf_getline(char **lineptr, size_t *n, FILE *stream)
{
#define MIN_CHUNK 16	/* Buffer is increased by this size each time as required */
	size_t i = 0;

	if (!*lineptr) {
		*n = MIN_CHUNK;
		*lineptr = malloc(*n);
		if (!*lineptr)
			return -1;
	}

	(*lineptr)[0] = fgetc(stream);
	while ((*lineptr)[i] != '\n') {
		(*lineptr)[++i] = fgetc(stream);
		if (feof(stream)) {
			(*lineptr)[0] = 0;
			return -1;
		}
		if ((i + 2) > *n) {
			*n += MIN_CHUNK;
			*lineptr = realloc(*lineptr, *n);
		}
	}

	(*lineptr)[++i] = 0;

	return sizeof(*lineptr);
}

#define SVFP_CMD_INC_CNT 1024
static int svf_read_command_from_file(FILE *fd)
{
	unsigned char ch;
	int i = 0;
	size_t cmd_pos = 0;
	int cmd_ok = 0, slash = 0;

	if (svf_getline(&svf_read_line, &svf_read_line_size, svf_fd) <= 0)
		return ERROR_FAIL;
	svf_line_number++;
	ch = svf_read_line[0];
	while (!cmd_ok && (ch != 0)) {
		switch (ch) {
			case '!':
				slash = 0;
				if (svf_getline(&svf_read_line, &svf_read_line_size, svf_fd) <= 0)
					return ERROR_FAIL;
				svf_line_number++;
				i = -1;
				break;
			case '/':
				if (++slash == 2) {
					slash = 0;
					if (svf_getline(&svf_read_line, &svf_read_line_size,
						svf_fd) <= 0)
						return ERROR_FAIL;
					svf_line_number++;
					i = -1;
				}
				break;
			case ';':
				slash = 0;
				cmd_ok = 1;
				break;
			case '\n':
				svf_line_number++;
				if (svf_getline(&svf_read_line, &svf_read_line_size, svf_fd) <= 0)
					return ERROR_FAIL;
				i = -1;
				/* fallthrough */
			case '\r':
				slash = 0;
				/* Don't save '\r' and '\n' if no data is parsed */
				if (!cmd_pos)
					break;
				/* fallthrough */
			default:
				/* The parsing code currently expects a space
				 * before parentheses -- "TDI (123)".  Also a
				 * space afterwards -- "TDI (123) TDO(456)".
				 * But such spaces are optional... instead of
				 * parser updates, cope with that by adding the
				 * spaces as needed.
				 *
				 * Ensure there are 3 bytes available, for:
				 *  - current character
				 *  - added space.
				 *  - terminating NUL ('\0')
				 */
				if (cmd_pos + 3 > svf_command_buffer_size) {
					svf_command_buffer = realloc(svf_command_buffer, cmd_pos + 3);
					svf_command_buffer_size = cmd_pos + 3;
					if (!svf_command_buffer) {
						LOG_ERROR("not enough memory");
						return ERROR_FAIL;
					}
				}

				/* insert a space before '(' */
				if ('(' == ch)
					svf_command_buffer[cmd_pos++] = ' ';

				svf_command_buffer[cmd_pos++] = (char)toupper(ch);

				/* insert a space after ')' */
				if (')' == ch)
					svf_command_buffer[cmd_pos++] = ' ';
				break;
		}
		ch = svf_read_line[++i];
	}

	if (cmd_ok) {
		svf_command_buffer[cmd_pos] = '\0';
		return ERROR_OK;
	} else
		return ERROR_FAIL;
}

static int svf_parse_cmd_string(char *str, int len, char **argus, int *num_of_argu)
{
	int pos = 0, num = 0, space_found = 1, in_bracket = 0;

	while (pos < len) {
		switch (str[pos]) {
			case '!':
			case '/':
				LOG_ERROR("fail to parse svf command");
				return ERROR_FAIL;
			case '(':
				in_bracket = 1;
				goto parse_char;
			case ')':
				in_bracket = 0;
				goto parse_char;
			default:
parse_char:
				if (!in_bracket && isspace((int) str[pos])) {
					space_found = 1;
					str[pos] = '\0';
				} else if (space_found) {
					argus[num++] = &str[pos];
					space_found = 0;
				}
				break;
		}
		pos++;
	}

	if (num == 0)
		return ERROR_FAIL;

	*num_of_argu = num;

	return ERROR_OK;
}

bool svf_tap_state_is_stable(tap_state_t state)
{
	return (state == TAP_RESET) || (state == TAP_IDLE)
			|| (state == TAP_DRPAUSE) || (state == TAP_IRPAUSE);
}

static int svf_find_string_in_array(char *str, char **strs, int num_of_element)
{
	int i;

	for (i = 0; i < num_of_element; i++) {
		if (!strcmp(str, strs[i]))
			return i;
	}
	return 0xFF;
}

static int svf_adjust_array_length(uint8_t **arr, int orig_bit_len, int new_bit_len)
{
	int new_byte_len = (new_bit_len + 7) >> 3;

	if ((!*arr) || (((orig_bit_len + 7) >> 3) < ((new_bit_len + 7) >> 3))) {
		free(*arr);
		*arr = calloc(1, new_byte_len);
		if (!*arr) {
			LOG_ERROR("not enough memory");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int svf_set_padding(struct svf_xxr_para *para, int len, unsigned char tdi)
{
	int error = ERROR_OK;
	error |= svf_adjust_array_length(&para->tdi, para->len, len);
	memset(para->tdi, tdi, (len + 7) >> 3);
	error |= svf_adjust_array_length(&para->tdo, para->len, len);
	error |= svf_adjust_array_length(&para->mask, para->len, len);
	para->len = len;
	para->data_mask = XXR_TDI;

	return error;
}

static int svf_copy_hexstring_to_binary(char *str, uint8_t **bin, int orig_bit_len, int bit_len)
{
	int i, str_len = strlen(str), str_hbyte_len = (bit_len + 3) >> 2;
	uint8_t ch = 0;

	if (svf_adjust_array_length(bin, orig_bit_len, bit_len) != ERROR_OK) {
		LOG_ERROR("fail to adjust length of array");
		return ERROR_FAIL;
	}

	/* fill from LSB (end of str) to MSB (beginning of str) */
	for (i = 0; i < str_hbyte_len; i++) {
		ch = 0;
		while (str_len > 0) {
			ch = str[--str_len];

			/* Skip whitespace.  The SVF specification (rev E) is
			 * deficient in terms of basic lexical issues like
			 * where whitespace is allowed.  Long bitstrings may
			 * require line ends for correctness, since there is
			 * a hard limit on line length.
			 */
			if (!isspace(ch)) {
				if ((ch >= '0') && (ch <= '9')) {
					ch = ch - '0';
					break;
				} else if ((ch >= 'A') && (ch <= 'F')) {
					ch = ch - 'A' + 10;
					break;
				} else {
					LOG_ERROR("invalid hex string");
					return ERROR_FAIL;
				}
			}

			ch = 0;
		}

		/* write bin */
		if (i % 2) {
			/* MSB */
			(*bin)[i / 2] |= ch << 4;
		} else {
			/* LSB */
			(*bin)[i / 2] = 0;
			(*bin)[i / 2] |= ch;
		}
	}

	/* consume optional leading '0' MSBs or whitespace */
	while (str_len > 0 && ((str[str_len - 1] == '0')
			|| isspace((int) str[str_len - 1])))
		str_len--;

	/* check validity: we must have consumed everything */
	if (str_len > 0 || (ch & ~((2 << ((bit_len - 1) % 4)) - 1)) != 0) {
		LOG_ERROR("value exceeds length");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int svf_check_tdo(void)
{
	int i, len, index_var;

	for (i = 0; i < svf_check_tdo_para_index; i++) {
		index_var = svf_check_tdo_para[i].buffer_offset;
		len = svf_check_tdo_para[i].bit_len;
		if ((svf_check_tdo_para[i].enabled)
				&& buf_cmp_mask(&svf_tdi_buffer[index_var], &svf_tdo_buffer[index_var],
				&svf_mask_buffer[index_var], len)) {
			LOG_ERROR("tdo check error at line %d",
				svf_check_tdo_para[i].line_num);
			SVF_BUF_LOG(ERROR, &svf_tdi_buffer[index_var], len, "READ");
			SVF_BUF_LOG(ERROR, &svf_tdo_buffer[index_var], len, "WANT");
			SVF_BUF_LOG(ERROR, &svf_mask_buffer[index_var], len, "MASK");

			if (svf_ignore_error == 0)
				return ERROR_FAIL;
			else
				svf_ignore_error++;
		}
	}
	svf_check_tdo_para_index = 0;

	return ERROR_OK;
}

static int svf_add_check_para(uint8_t enabled, int buffer_offset, int bit_len)
{
	if (svf_check_tdo_para_index >= SVF_CHECK_TDO_PARA_SIZE) {
		LOG_ERROR("toooooo many operation undone");
		return ERROR_FAIL;
	}

	svf_check_tdo_para[svf_check_tdo_para_index].line_num = svf_line_number;
	svf_check_tdo_para[svf_check_tdo_para_index].bit_len = bit_len;
	svf_check_tdo_para[svf_check_tdo_para_index].enabled = enabled;
	svf_check_tdo_para[svf_check_tdo_para_index].buffer_offset = buffer_offset;
	svf_check_tdo_para_index++;

	return ERROR_OK;
}

static int svf_execute_tap(void)
{
	if ((!svf_nil) && (jtag_execute_queue() != ERROR_OK))
		return ERROR_FAIL;
	else if (svf_check_tdo() != ERROR_OK)
		return ERROR_FAIL;

	svf_buffer_index = 0;

	return ERROR_OK;
}

static int svf_run_command(struct command_context *cmd_ctx, char *cmd_str)
{
	char *argus[256], command;
	int num_of_argu = 0, i;

	/* tmp variable */
	int i_tmp;

	/* for RUNTEST */
	int run_count;
	float min_time;
	/* for XXR */
	struct svf_xxr_para *xxr_para_tmp;
	uint8_t **pbuffer_tmp;
	struct scan_field field;
	/* for STATE */
	tap_state_t *path = NULL, state;
	/* flag padding commands skipped due to -tap command */
	int padding_command_skipped = 0;

	if (svf_parse_cmd_string(cmd_str, strlen(cmd_str), argus, &num_of_argu) != ERROR_OK)
		return ERROR_FAIL;

	/* NOTE: we're a bit loose here, because we ignore case in
	 * TAP state names (instead of insisting on uppercase).
	 */

	command = svf_find_string_in_array(argus[0],
			(char **)svf_command_name, ARRAY_SIZE(svf_command_name));
	switch (command) {
		case ENDDR:
		case ENDIR:
			if (num_of_argu != 2) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}

			i_tmp = tap_state_by_name(argus[1]);

			if (svf_tap_state_is_stable(i_tmp)) {
				if (command == ENDIR) {
					svf_para.ir_end_state = i_tmp;
					LOG_DEBUG("\tIR end_state = %s",
							tap_state_name(i_tmp));
				} else {
					svf_para.dr_end_state = i_tmp;
					LOG_DEBUG("\tDR end_state = %s",
							tap_state_name(i_tmp));
				}
			} else {
				LOG_ERROR("%s: %s is not a stable state",
						argus[0], argus[1]);
				return ERROR_FAIL;
			}
			break;
		case FREQUENCY:
			if ((num_of_argu != 1) && (num_of_argu != 3)) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}
			if (num_of_argu == 1) {
				/* TODO: set jtag speed to full speed */
				svf_para.frequency = 0;
			} else {
				if (strcmp(argus[2], "HZ")) {
					LOG_ERROR("HZ not found in FREQUENCY command");
					return ERROR_FAIL;
				}
				if (svf_execute_tap() != ERROR_OK)
					return ERROR_FAIL;
				svf_para.frequency = atof(argus[1]);
				/* TODO: set jtag speed to */
				if (svf_para.frequency > 0) {
					command_run_linef(cmd_ctx,
							"adapter speed %d",
							(int)svf_para.frequency / 1000);
					LOG_DEBUG("\tfrequency = %f", svf_para.frequency);
				}
			}
			break;
		case HDR:
			if (svf_tap_is_specified) {
				padding_command_skipped = 1;
				break;
			}
			xxr_para_tmp = &svf_para.hdr_para;
			goto xxr_common;
		case HIR:
			if (svf_tap_is_specified) {
				padding_command_skipped = 1;
				break;
			}
			xxr_para_tmp = &svf_para.hir_para;
			goto xxr_common;
		case TDR:
			if (svf_tap_is_specified) {
				padding_command_skipped = 1;
				break;
			}
			xxr_para_tmp = &svf_para.tdr_para;
			goto xxr_common;
		case TIR:
			if (svf_tap_is_specified) {
				padding_command_skipped = 1;
				break;
			}
			xxr_para_tmp = &svf_para.tir_para;
			goto xxr_common;
		case SDR:
			xxr_para_tmp = &svf_para.sdr_para;
			goto xxr_common;
		case SIR:
			xxr_para_tmp = &svf_para.sir_para;
			goto xxr_common;
xxr_common:
			/* XXR length [TDI (tdi)] [TDO (tdo)][MASK (mask)] [SMASK (smask)] */
			if ((num_of_argu > 10) || (num_of_argu % 2)) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}
			i_tmp = xxr_para_tmp->len;
			xxr_para_tmp->len = atoi(argus[1]);
			/* If we are to enlarge the buffers, all parts of xxr_para_tmp
			 * need to be freed */
			if (i_tmp < xxr_para_tmp->len) {
				free(xxr_para_tmp->tdi);
				xxr_para_tmp->tdi = NULL;
				free(xxr_para_tmp->tdo);
				xxr_para_tmp->tdo = NULL;
				free(xxr_para_tmp->mask);
				xxr_para_tmp->mask = NULL;
				free(xxr_para_tmp->smask);
				xxr_para_tmp->smask = NULL;
			}

			LOG_DEBUG("\tlength = %d", xxr_para_tmp->len);
			xxr_para_tmp->data_mask = 0;
			for (i = 2; i < num_of_argu; i += 2) {
				if ((strlen(argus[i + 1]) < 3) || (argus[i + 1][0] != '(') ||
				(argus[i + 1][strlen(argus[i + 1]) - 1] != ')')) {
					LOG_ERROR("data section error");
					return ERROR_FAIL;
				}
				argus[i + 1][strlen(argus[i + 1]) - 1] = '\0';
				/* TDI, TDO, MASK, SMASK */
				if (!strcmp(argus[i], "TDI")) {
					/* TDI */
					pbuffer_tmp = &xxr_para_tmp->tdi;
					xxr_para_tmp->data_mask |= XXR_TDI;
				} else if (!strcmp(argus[i], "TDO")) {
					/* TDO */
					pbuffer_tmp = &xxr_para_tmp->tdo;
					xxr_para_tmp->data_mask |= XXR_TDO;
				} else if (!strcmp(argus[i], "MASK")) {
					/* MASK */
					pbuffer_tmp = &xxr_para_tmp->mask;
					xxr_para_tmp->data_mask |= XXR_MASK;
				} else if (!strcmp(argus[i], "SMASK")) {
					/* SMASK */
					pbuffer_tmp = &xxr_para_tmp->smask;
					xxr_para_tmp->data_mask |= XXR_SMASK;
				} else {
					LOG_ERROR("unknown parameter: %s", argus[i]);
					return ERROR_FAIL;
				}
				if (ERROR_OK !=
				svf_copy_hexstring_to_binary(&argus[i + 1][1], pbuffer_tmp, i_tmp,
					xxr_para_tmp->len)) {
					LOG_ERROR("fail to parse hex value");
					return ERROR_FAIL;
				}
				SVF_BUF_LOG(DEBUG, *pbuffer_tmp, xxr_para_tmp->len, argus[i]);
			}
			/* If a command changes the length of the last scan of the same type and the
			 * MASK parameter is absent, */
			/* the mask pattern used is all cares */
			if (!(xxr_para_tmp->data_mask & XXR_MASK) && (i_tmp != xxr_para_tmp->len)) {
				/* MASK not defined and length changed */
				if (ERROR_OK !=
				svf_adjust_array_length(&xxr_para_tmp->mask, i_tmp,
					xxr_para_tmp->len)) {
					LOG_ERROR("fail to adjust length of array");
					return ERROR_FAIL;
				}
				buf_set_ones(xxr_para_tmp->mask, xxr_para_tmp->len);
			}
			/* If TDO is absent, no comparison is needed, set the mask to 0 */
			if (!(xxr_para_tmp->data_mask & XXR_TDO)) {
				if (!xxr_para_tmp->tdo) {
					if (ERROR_OK !=
					svf_adjust_array_length(&xxr_para_tmp->tdo, i_tmp,
						xxr_para_tmp->len)) {
						LOG_ERROR("fail to adjust length of array");
						return ERROR_FAIL;
					}
				}
				if (!xxr_para_tmp->mask) {
					if (ERROR_OK !=
					svf_adjust_array_length(&xxr_para_tmp->mask, i_tmp,
						xxr_para_tmp->len)) {
						LOG_ERROR("fail to adjust length of array");
						return ERROR_FAIL;
					}
				}
				memset(xxr_para_tmp->mask, 0, (xxr_para_tmp->len + 7) >> 3);
			}
			/* do scan if necessary */
			if (command == SDR) {
				/* check buffer size first, reallocate if necessary */
				i = svf_para.hdr_para.len + svf_para.sdr_para.len +
						svf_para.tdr_para.len;
				if ((svf_buffer_size - svf_buffer_index) < ((i + 7) >> 3)) {
					/* reallocate buffer */
					if (svf_realloc_buffers(svf_buffer_index + ((i + 7) >> 3)) != ERROR_OK) {
						LOG_ERROR("not enough memory");
						return ERROR_FAIL;
					}
				}

				/* assemble dr data */
				i = 0;
				buf_set_buf(svf_para.hdr_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.hdr_para.len);
				i += svf_para.hdr_para.len;
				buf_set_buf(svf_para.sdr_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.sdr_para.len);
				i += svf_para.sdr_para.len;
				buf_set_buf(svf_para.tdr_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.tdr_para.len);
				i += svf_para.tdr_para.len;

				/* add check data */
				if (svf_para.sdr_para.data_mask & XXR_TDO) {
					/* assemble dr mask data */
					i = 0;
					buf_set_buf(svf_para.hdr_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.hdr_para.len);
					i += svf_para.hdr_para.len;
					buf_set_buf(svf_para.sdr_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.sdr_para.len);
					i += svf_para.sdr_para.len;
					buf_set_buf(svf_para.tdr_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.tdr_para.len);

					/* assemble dr check data */
					i = 0;
					buf_set_buf(svf_para.hdr_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.hdr_para.len);
					i += svf_para.hdr_para.len;
					buf_set_buf(svf_para.sdr_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.sdr_para.len);
					i += svf_para.sdr_para.len;
					buf_set_buf(svf_para.tdr_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.tdr_para.len);
					i += svf_para.tdr_para.len;

					svf_add_check_para(1, svf_buffer_index, i);
				} else
					svf_add_check_para(0, svf_buffer_index, i);
				field.num_bits = i;
				field.out_value = &svf_tdi_buffer[svf_buffer_index];
				field.in_value = (xxr_para_tmp->data_mask & XXR_TDO) ? &svf_tdi_buffer[svf_buffer_index] : NULL;
				if (!svf_nil) {
					/* NOTE:  doesn't use SVF-specified state paths */
					jtag_add_plain_dr_scan(field.num_bits,
							field.out_value,
							field.in_value,
							svf_para.dr_end_state);
				}

				svf_buffer_index += (i + 7) >> 3;
			} else if (command == SIR) {
				/* check buffer size first, reallocate if necessary */
				i = svf_para.hir_para.len + svf_para.sir_para.len +
						svf_para.tir_para.len;
				if ((svf_buffer_size - svf_buffer_index) < ((i + 7) >> 3)) {
					if (svf_realloc_buffers(svf_buffer_index + ((i + 7) >> 3)) != ERROR_OK) {
						LOG_ERROR("not enough memory");
						return ERROR_FAIL;
					}
				}

				/* assemble ir data */
				i = 0;
				buf_set_buf(svf_para.hir_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.hir_para.len);
				i += svf_para.hir_para.len;
				buf_set_buf(svf_para.sir_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.sir_para.len);
				i += svf_para.sir_para.len;
				buf_set_buf(svf_para.tir_para.tdi,
						0,
						&svf_tdi_buffer[svf_buffer_index],
						i,
						svf_para.tir_para.len);
				i += svf_para.tir_para.len;

				/* add check data */
				if (svf_para.sir_para.data_mask & XXR_TDO) {
					/* assemble dr mask data */
					i = 0;
					buf_set_buf(svf_para.hir_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.hir_para.len);
					i += svf_para.hir_para.len;
					buf_set_buf(svf_para.sir_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.sir_para.len);
					i += svf_para.sir_para.len;
					buf_set_buf(svf_para.tir_para.mask,
							0,
							&svf_mask_buffer[svf_buffer_index],
							i,
							svf_para.tir_para.len);

					/* assemble dr check data */
					i = 0;
					buf_set_buf(svf_para.hir_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.hir_para.len);
					i += svf_para.hir_para.len;
					buf_set_buf(svf_para.sir_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.sir_para.len);
					i += svf_para.sir_para.len;
					buf_set_buf(svf_para.tir_para.tdo,
							0,
							&svf_tdo_buffer[svf_buffer_index],
							i,
							svf_para.tir_para.len);
					i += svf_para.tir_para.len;

					svf_add_check_para(1, svf_buffer_index, i);
				} else
					svf_add_check_para(0, svf_buffer_index, i);
				field.num_bits = i;
				field.out_value = &svf_tdi_buffer[svf_buffer_index];
				field.in_value = (xxr_para_tmp->data_mask & XXR_TDO) ? &svf_tdi_buffer[svf_buffer_index] : NULL;
				if (!svf_nil) {
					/* NOTE:  doesn't use SVF-specified state paths */
					jtag_add_plain_ir_scan(field.num_bits,
							field.out_value,
							field.in_value,
							svf_para.ir_end_state);
				}

				svf_buffer_index += (i + 7) >> 3;
			}
			break;
		case PIO:
		case PIOMAP:
			LOG_ERROR("PIO and PIOMAP are not supported");
			return ERROR_FAIL;
		case RUNTEST:
			/* RUNTEST [run_state] run_count run_clk [min_time SEC [MAXIMUM max_time
			 * SEC]] [ENDSTATE end_state] */
			/* RUNTEST [run_state] min_time SEC [MAXIMUM max_time SEC] [ENDSTATE
			 * end_state] */
			if ((num_of_argu < 3) || (num_of_argu > 11)) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}
			/* init */
			run_count = 0;
			min_time = 0;
			i = 1;

			/* run_state */
			i_tmp = tap_state_by_name(argus[i]);
			if (i_tmp != TAP_INVALID) {
				if (svf_tap_state_is_stable(i_tmp)) {
					svf_para.runtest_run_state = i_tmp;

					/* When a run_state is specified, the new
					 * run_state becomes the default end_state.
					 */
					svf_para.runtest_end_state = i_tmp;
					LOG_DEBUG("\trun_state = %s", tap_state_name(i_tmp));
					i++;
				} else {
					LOG_ERROR("%s: %s is not a stable state", argus[0], tap_state_name(i_tmp));
					return ERROR_FAIL;
				}
			}

			/* run_count run_clk */
			if (((i + 2) <= num_of_argu) && strcmp(argus[i + 1], "SEC")) {
				if (!strcmp(argus[i + 1], "TCK")) {
					/* clock source is TCK */
					run_count = atoi(argus[i]);
					LOG_DEBUG("\trun_count@TCK = %d", run_count);
				} else {
					LOG_ERROR("%s not supported for clock", argus[i + 1]);
					return ERROR_FAIL;
				}
				i += 2;
			}
			/* min_time SEC */
			if (((i + 2) <= num_of_argu) && !strcmp(argus[i + 1], "SEC")) {
				min_time = atof(argus[i]);
				LOG_DEBUG("\tmin_time = %fs", min_time);
				i += 2;
			}
			/* MAXIMUM max_time SEC */
			if (((i + 3) <= num_of_argu) &&
			!strcmp(argus[i], "MAXIMUM") && !strcmp(argus[i + 2], "SEC")) {
				float max_time = 0;
				max_time = atof(argus[i + 1]);
				LOG_DEBUG("\tmax_time = %fs", max_time);
				i += 3;
			}
			/* ENDSTATE end_state */
			if (((i + 2) <= num_of_argu) && !strcmp(argus[i], "ENDSTATE")) {
				i_tmp = tap_state_by_name(argus[i + 1]);

				if (svf_tap_state_is_stable(i_tmp)) {
					svf_para.runtest_end_state = i_tmp;
					LOG_DEBUG("\tend_state = %s", tap_state_name(i_tmp));
				} else {
					LOG_ERROR("%s: %s is not a stable state", argus[0], tap_state_name(i_tmp));
					return ERROR_FAIL;
				}
				i += 2;
			}

			/* all parameter should be parsed */
			if (i == num_of_argu) {
#if 1
				/* FIXME handle statemove failures */
				uint32_t min_usec = 1000000 * min_time;

				/* enter into run_state if necessary */
				if (cmd_queue_cur_state != svf_para.runtest_run_state)
					svf_add_statemove(svf_para.runtest_run_state);

				/* add clocks and/or min wait */
				if (run_count > 0) {
					if (!svf_nil)
						jtag_add_clocks(run_count);
				}

				if (min_usec > 0) {
					if (!svf_nil)
						jtag_add_sleep(min_usec);
				}

				/* move to end_state if necessary */
				if (svf_para.runtest_end_state != svf_para.runtest_run_state)
					svf_add_statemove(svf_para.runtest_end_state);

#else
				if (svf_para.runtest_run_state != TAP_IDLE) {
					LOG_ERROR("cannot runtest in %s state",
							tap_state_name(svf_para.runtest_run_state));
					return ERROR_FAIL;
				}

				if (!svf_nil)
					jtag_add_runtest(run_count, svf_para.runtest_end_state);
#endif
			} else {
				LOG_ERROR("fail to parse parameter of RUNTEST, %d out of %d is parsed",
						i,
						num_of_argu);
				return ERROR_FAIL;
			}
			break;
		case STATE:
			/* STATE [pathstate1 [pathstate2 ...[pathstaten]]] stable_state */
			if (num_of_argu < 2) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}
			if (num_of_argu > 2) {
				/* STATE pathstate1 ... stable_state */
				path = malloc((num_of_argu - 1) * sizeof(tap_state_t));
				if (!path) {
					LOG_ERROR("not enough memory");
					return ERROR_FAIL;
				}
				num_of_argu--;	/* num of path */
				i_tmp = 1;		/* path is from parameter 1 */
				for (i = 0; i < num_of_argu; i++, i_tmp++) {
					path[i] = tap_state_by_name(argus[i_tmp]);
					if (path[i] == TAP_INVALID) {
						LOG_ERROR("%s: %s is not a valid state", argus[0], argus[i_tmp]);
						free(path);
						return ERROR_FAIL;
					}
					/* OpenOCD refuses paths containing TAP_RESET */
					if (path[i] == TAP_RESET) {
						/* FIXME last state MUST be stable! */
						if (i > 0) {
							if (!svf_nil)
								jtag_add_pathmove(i, path);
						}
						if (!svf_nil)
							jtag_add_tlr();
						num_of_argu -= i + 1;
						i = -1;
					}
				}
				if (num_of_argu > 0) {
					/* execute last path if necessary */
					if (svf_tap_state_is_stable(path[num_of_argu - 1])) {
						/* last state MUST be stable state */
						if (!svf_nil)
							jtag_add_pathmove(num_of_argu, path);
						LOG_DEBUG("\tmove to %s by path_move",
								tap_state_name(path[num_of_argu - 1]));
					} else {
						LOG_ERROR("%s: %s is not a stable state",
								argus[0],
								tap_state_name(path[num_of_argu - 1]));
						free(path);
						return ERROR_FAIL;
					}
				}

				free(path);
				path = NULL;
			} else {
				/* STATE stable_state */
				state = tap_state_by_name(argus[1]);
				if (svf_tap_state_is_stable(state)) {
					LOG_DEBUG("\tmove to %s by svf_add_statemove",
							tap_state_name(state));
					/* FIXME handle statemove failures */
					svf_add_statemove(state);
				} else {
					LOG_ERROR("%s: %s is not a stable state",
							argus[0], tap_state_name(state));
					return ERROR_FAIL;
				}
			}
			break;
		case TRST:
			/* TRST trst_mode */
			if (num_of_argu != 2) {
				LOG_ERROR("invalid parameter of %s", argus[0]);
				return ERROR_FAIL;
			}
			if (svf_para.trst_mode != TRST_ABSENT) {
				if (svf_execute_tap() != ERROR_OK)
					return ERROR_FAIL;
				i_tmp = svf_find_string_in_array(argus[1],
						(char **)svf_trst_mode_name,
						ARRAY_SIZE(svf_trst_mode_name));
				switch (i_tmp) {
				case TRST_ON:
					if (!svf_nil)
						jtag_add_reset(1, 0);
					break;
				case TRST_Z:
				case TRST_OFF:
					if (!svf_nil)
						jtag_add_reset(0, 0);
					break;
				case TRST_ABSENT:
					break;
				default:
					LOG_ERROR("unknown TRST mode: %s", argus[1]);
					return ERROR_FAIL;
				}
				svf_para.trst_mode = i_tmp;
				LOG_DEBUG("\ttrst_mode = %s", svf_trst_mode_name[svf_para.trst_mode]);
			} else {
				LOG_ERROR("can not accept TRST command if trst_mode is ABSENT");
				return ERROR_FAIL;
			}
			break;
		default:
			LOG_ERROR("invalid svf command: %s", argus[0]);
			return ERROR_FAIL;
	}

	if (!svf_quiet) {
		if (padding_command_skipped)
			LOG_USER("(Above Padding command skipped, as per -tap argument)");
	}

	if (debug_level >= LOG_LVL_DEBUG) {
		/* for convenient debugging, execute tap if possible */
		if ((svf_buffer_index > 0) &&
				(((command != STATE) && (command != RUNTEST)) ||
						((command == STATE) && (num_of_argu == 2)))) {
			if (svf_execute_tap() != ERROR_OK)
				return ERROR_FAIL;

			/* output debug info */
			if ((command == SIR) || (command == SDR))
				SVF_BUF_LOG(DEBUG, svf_tdi_buffer, svf_check_tdo_para[0].bit_len, "TDO read");
		}
	} else {
		/* for fast executing, execute tap if necessary */
		/* half of the buffer is for the next command */
		if (((svf_buffer_index >= SVF_MAX_BUFFER_SIZE_TO_COMMIT) ||
				(svf_check_tdo_para_index >= SVF_CHECK_TDO_PARA_SIZE / 2)) &&
				(((command != STATE) && (command != RUNTEST)) ||
						((command == STATE) && (num_of_argu == 2))))
			return svf_execute_tap();
	}

	return ERROR_OK;
}

static const struct command_registration svf_command_handlers[] = {
	{
		.name = "svf",
		.handler = handle_svf_command,
		.mode = COMMAND_EXEC,
		.help = "Runs a SVF file.",
		.usage = "[-tap device.tap] <file> [quiet] [nil] [progress] [ignore_error]",
	},
	COMMAND_REGISTRATION_DONE
};

int svf_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, svf_command_handlers);
}
