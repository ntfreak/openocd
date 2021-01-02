/***************************************************************************
 *   Copyright (C) 2018 by Liviu Ionescu                                   *
 *   <ilg@livius.net>                                                      *
 *                                                                         *
 *   Copyright (C) 2018 by Marvell Technology Group Ltd.                   *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
 *                                                                         *
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
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

/**
 * @file
 * Common ARM semihosting support.
 *
 * Semihosting enables code running on a target to use some of the I/O
 * facilities on the host computer. The target application must be linked
 * against a library that forwards operation requests by using  an
 * instruction trapped by the debugger.
 *
 * Details can be found in
 * "Semihosting for AArch32 and AArch64, Release 2.0"
 * https://static.docs.arm.com/100863/0200/semihosting.pdf
 * from ARM Ltd.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "semihosting_common.h"

#include <helper/binarybuffer.h>
#include <helper/log.h>
#include <sys/stat.h>

static const int open_modeflags[12] = {
	O_RDONLY,
	O_RDONLY | O_BINARY,
	O_RDWR,
	O_RDWR | O_BINARY,
	O_WRONLY | O_CREAT | O_TRUNC,
	O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
	O_RDWR | O_CREAT | O_TRUNC,
	O_RDWR | O_CREAT | O_TRUNC | O_BINARY,
	O_WRONLY | O_CREAT | O_APPEND,
	O_WRONLY | O_CREAT | O_APPEND | O_BINARY,
	O_RDWR | O_CREAT | O_APPEND,
	O_RDWR | O_CREAT | O_APPEND | O_BINARY
};

static int semihosting_common_fileio_info(struct target *target,
	struct gdb_fileio_info *fileio_info);
static int semihosting_common_fileio_end(struct target *target, int result,
	int fileio_errno, bool ctrl_c);

static int semihosting_read_fields(struct target *target, size_t number,
	uint8_t *fields);
static int semihosting_write_fields(struct target *target, size_t number,
	uint8_t *fields);
static uint64_t semihosting_get_field(struct target *target, size_t index,
	uint8_t *fields);
static void semihosting_set_field(struct target *target, uint64_t value,
	size_t index,
	uint8_t *fields);

/* Attempts to include gdb_server.h failed. */
extern int gdb_actual_connections;

/**
 * Initialize common semihosting support.
 *
 * @param target Pointer to the target to initialize.
 * @param setup
 * @param post_result
 * @return An error status if there is a problem during initialization.
 */
int semihosting_common_init(struct target *target, void *setup,
	void *post_result)
{
	LOG_DEBUG(" ");

	target->fileio_info = malloc(sizeof(*target->fileio_info));
	if (target->fileio_info == NULL) {
		LOG_ERROR("out of memory");
		return ERROR_FAIL;
	}
	memset(target->fileio_info, 0, sizeof(*target->fileio_info));

	struct semihosting *semihosting;
	semihosting = malloc(sizeof(*target->semihosting));
	if (semihosting == NULL) {
		LOG_ERROR("out of memory");
		return ERROR_FAIL;
	}

	semihosting->is_active = false;
	semihosting->is_fileio = false;
	semihosting->hit_fileio = false;
	semihosting->is_resumable = false;
	semihosting->has_resumable_exit = false;
	semihosting->word_size_bytes = 0;
	semihosting->op = -1;
	semihosting->param = 0;
	semihosting->result = -1;
	semihosting->sys_errno = -1;
	semihosting->cmdline = NULL;

	/* If possible, update it in setup(). */
	semihosting->setup_time = clock();

	semihosting->setup = setup;
	semihosting->post_result = post_result;

	target->semihosting = semihosting;

	target->type->get_gdb_fileio_info = semihosting_common_fileio_info;
	target->type->gdb_fileio_end = semihosting_common_fileio_end;

	return ERROR_OK;
}

/**
 * Portable implementation of ARM semihosting calls.
 * Performs the currently pending semihosting operation
 * encoded in target->semihosting.
 */
int semihosting_common(struct target *target)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		/* Silently ignore if the semihosting field was not set. */
		return ERROR_OK;
	}

	struct gdb_fileio_info *fileio_info = target->fileio_info;

	/*
	 * By default return an error.
	 * The actual result must be set by each function
	 */
	semihosting->result = -1;

	/* Most operations are resumable, except the two exit calls. */
	semihosting->is_resumable = true;

	int retval;

	/* Enough space to hold 4 long words. */
	uint8_t fields[4*8];

	LOG_DEBUG("op=0x%x, param=0x%" PRIx64, (int)semihosting->op,
		semihosting->param);

	switch (semihosting->op) {

		case SEMIHOSTING_SYS_CLOCK:	/* 0x10 */
			/*
			 * Returns the number of centiseconds (hundredths of a second)
			 * since the execution started.
			 *
			 * Values returned can be of limited use for some benchmarking
			 * purposes because of communication overhead or other
			 * agent-specific factors. For example, with a debug hardware
			 * unit the request is passed back to the host for execution.
			 * This can lead to unpredictable delays in transmission and
			 * process scheduling.
			 *
			 * Use this function to calculate time intervals, by calculating
			 * differences between intervals with and without the code
			 * sequence to be timed.
			 *
			 * Entry
			 * The PARAMETER REGISTER must contain 0. There are no other
			 * parameters.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - The number of centiseconds since some arbitrary start
			 * point, if the call is successful.
			 * - –1 if the call is not successful. For example, because
			 * of a communications error.
			 */
		{
			clock_t delta = clock() - semihosting->setup_time;

			semihosting->result = delta / (CLOCKS_PER_SEC / 100);
		}
		break;

		case SEMIHOSTING_SYS_CLOSE:	/* 0x02 */
			/*
			 * Closes a file on the host system. The handle must reference
			 * a file that was opened with SYS_OPEN.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * one-field argument block:
			 * - field 1 Contains a handle for an open file.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - 0 if the call is successful
			 * - –1 if the call is not successful.
			 */
			retval = semihosting_read_fields(target, 1, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				int fd = semihosting_get_field(target, 0, fields);
				if (semihosting->is_fileio) {
					if (fd == 0 || fd == 1 || fd == 2) {
						semihosting->result = 0;
						break;
					}
					semihosting->hit_fileio = true;
					fileio_info->identifier = "close";
					fileio_info->param_1 = fd;
				} else {
					semihosting->result = close(fd);
					semihosting->sys_errno = errno;

					LOG_DEBUG("close(%d)=%d", fd, (int)semihosting->result);
				}
			}
			break;

		case SEMIHOSTING_SYS_ERRNO:	/* 0x13 */
			/*
			 * Returns the value of the C library errno variable that is
			 * associated with the semihosting implementation. The errno
			 * variable can be set by a number of C library semihosted
			 * functions, including:
			 * - SYS_REMOVE
			 * - SYS_OPEN
			 * - SYS_CLOSE
			 * - SYS_READ
			 * - SYS_WRITE
			 * - SYS_SEEK.
			 *
			 * Whether errno is set or not, and to what value, is entirely
			 * host-specific, except where the ISO C standard defines the
			 * behavior.
			 *
			 * Entry
			 * There are no parameters. The PARAMETER REGISTER must be 0.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains the value of the C
			 * library errno variable.
			 */
			semihosting->result = semihosting->sys_errno;
			break;

		case SEMIHOSTING_SYS_EXIT:	/* 0x18 */
			/*
			 * Note: SYS_EXIT was called angel_SWIreason_ReportException in
			 * previous versions of the documentation.
			 *
			 * An application calls this operation to report an exception
			 * to the debugger directly. The most common use is to report
			 * that execution has completed, using ADP_Stopped_ApplicationExit.
			 *
			 * Note: This semihosting operation provides no means for 32-bit
			 * callers to indicate an application exit with a specified exit
			 * code. Semihosting callers may prefer to check for the presence
			 * of the SH_EXT_EXTENDED_REPORT_EXCEPTION extension and use
			 * the SYS_REPORT_EXCEPTION_EXTENDED operation instead, if it
			 * is available.
			 *
			 * Entry (32-bit)
			 * On entry, the PARAMETER register is set to a reason code
			 * describing the cause of the trap. Not all semihosting client
			 * implementations will necessarily trap every corresponding
			 * event. Important reason codes are:
			 *
			 * - ADP_Stopped_ApplicationExit 0x20026
			 * - ADP_Stopped_RunTimeErrorUnknown 0x20023
			 *
			 * Entry (64-bit)
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * two-field argument block:
			 * - field 1 The exception type, which is one of the set of
			 * reason codes in the above tables.
			 * - field 2 A subcode, whose meaning depends on the reason
			 * code in field 1.
			 * In particular, if field 1 is ADP_Stopped_ApplicationExit
			 * then field 2 is an exit status code, as passed to the C
			 * standard library exit() function. A simulator receiving
			 * this request must notify a connected debugger, if present,
			 * and then exit with the specified status.
			 *
			 * Return
			 * No return is expected from these calls. However, it is
			 * possible for the debugger to request that the application
			 * continues by performing an RDI_Execute request or equivalent.
			 * In this case, execution continues with the registers as they
			 * were on entry to the operation, or as subsequently modified
			 * by the debugger.
			 */
			if (semihosting->word_size_bytes == 8) {
				retval = semihosting_read_fields(target, 2, fields);
				if (retval != ERROR_OK)
					return retval;
				else {
					int type = semihosting_get_field(target, 0, fields);
					int code = semihosting_get_field(target, 1, fields);

					if (type == ADP_STOPPED_APPLICATION_EXIT) {
						if (!gdb_actual_connections)
							exit(code);
						else {
							fprintf(stderr,
								"semihosting: *** application exited with %d ***\n",
								code);
						}
					} else {
						fprintf(stderr,
							"semihosting: application exception %#x\n",
							type);
					}
				}
			} else {
				if (semihosting->param == ADP_STOPPED_APPLICATION_EXIT) {
					if (!gdb_actual_connections)
						exit(0);
					else {
						fprintf(stderr,
							"semihosting: *** application exited normally ***\n");
					}
				} else if (semihosting->param == ADP_STOPPED_RUN_TIME_ERROR) {
					/* Chosen more or less arbitrarily to have a nicer message,
					 * otherwise all other return the same exit code 1. */
					if (!gdb_actual_connections)
						exit(1);
					else {
						fprintf(stderr,
							"semihosting: *** application exited with error ***\n");
					}
				} else {
					if (!gdb_actual_connections)
						exit(1);
					else {
						fprintf(stderr,
							"semihosting: application exception %#x\n",
							(unsigned) semihosting->param);
					}
				}
			}
			if (!semihosting->has_resumable_exit) {
				semihosting->is_resumable = false;
				return target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
			break;

		case SEMIHOSTING_SYS_EXIT_EXTENDED:	/* 0x20 */
			/*
			 * This operation is only supported if the semihosting extension
			 * SH_EXT_EXIT_EXTENDED is implemented. SH_EXT_EXIT_EXTENDED is
			 * reported using feature byte 0, bit 0. If this extension is
			 * supported, then the implementation provides a means to
			 * report a normal exit with a nonzero exit status in both 32-bit
			 * and 64-bit semihosting APIs.
			 *
			 * The implementation must provide the semihosting call
			 * SYS_EXIT_EXTENDED for both A64 and A32/T32 semihosting APIs.
			 *
			 * SYS_EXIT_EXTENDED is used by an application to report an
			 * exception or exit to the debugger directly. The most common
			 * use is to report that execution has completed, using
			 * ADP_Stopped_ApplicationExit.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * two-field argument block:
			 * - field 1 The exception type, which should be one of the set
			 * of reason codes that are documented for the SYS_EXIT
			 * (0x18) call. For example, ADP_Stopped_ApplicationExit.
			 * - field 2 A subcode, whose meaning depends on the reason
			 * code in field 1. In particular, if field 1 is
			 * ADP_Stopped_ApplicationExit then field 2 is an exit status
			 * code, as passed to the C standard library exit() function.
			 * A simulator receiving this request must notify a connected
			 * debugger, if present, and then exit with the specified status.
			 *
			 * Return
			 * No return is expected from these calls.
			 *
			 * For the A64 API, this call is identical to the behavior of
			 * the mandatory SYS_EXIT (0x18) call. If this extension is
			 * supported, then both calls must be implemented.
			 */
			retval = semihosting_read_fields(target, 2, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				int type = semihosting_get_field(target, 0, fields);
				int code = semihosting_get_field(target, 1, fields);

				if (type == ADP_STOPPED_APPLICATION_EXIT) {
					if (!gdb_actual_connections)
						exit(code);
					else {
						fprintf(stderr,
							"semihosting: *** application exited with %d ***\n",
							code);
					}
				} else {
					fprintf(stderr, "semihosting: exception %#x\n",
						type);
				}
			}
			if (!semihosting->has_resumable_exit) {
				semihosting->is_resumable = false;
				return target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
			break;

		case SEMIHOSTING_SYS_FLEN:	/* 0x0C */
			/*
			 * Returns the length of a specified file.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * one-field argument block:
			 * - field 1 A handle for a previously opened, seekable file
			 * object.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - The current length of the file object, if the call is
			 * successful.
			 * - –1 if an error occurs.
			 */
			if (semihosting->is_fileio) {
				semihosting->result = -1;
				semihosting->sys_errno = EINVAL;
			}
			retval = semihosting_read_fields(target, 1, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				int fd = semihosting_get_field(target, 0, fields);
				struct stat buf;
				semihosting->result = fstat(fd, &buf);
				if (semihosting->result == -1) {
					semihosting->sys_errno = errno;
					LOG_DEBUG("fstat(%d)=%d", fd, (int)semihosting->result);
					break;
				}
				LOG_DEBUG("fstat(%d)=%d", fd, (int)semihosting->result);
				semihosting->result = buf.st_size;
			}
			break;

		case SEMIHOSTING_SYS_GET_CMDLINE:	/* 0x15 */
			/*
			 * Returns the command line that is used for the call to the
			 * executable, that is, argc and argv.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER points to a two-field data
			 * block to be used for returning the command string and its length:
			 * - field 1 A pointer to a buffer of at least the size that is
			 * specified in field 2.
			 * - field 2 The length of the buffer in bytes.
			 *
			 * Return
			 * On exit:
			 * If the call is successful, then the RETURN REGISTER contains 0,
			 * the PARAMETER REGISTER is unchanged, and the data block is
			 * updated as follows:
			 * - field 1 A pointer to a null-terminated string of the command
			 * line.
			 * - field 2 The length of the string in bytes.
			 * If the call is not successful, then the RETURN REGISTER
			 * contains -1.
			 *
			 * Note: The semihosting implementation might impose limits on
			 * the maximum length of the string that can be transferred.
			 * However, the implementation must be able to support a
			 * command-line length of at least 80 bytes.
			 */
			retval = semihosting_read_fields(target, 2, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				uint64_t addr = semihosting_get_field(target, 0, fields);
				size_t size = semihosting_get_field(target, 1, fields);

				char *arg = semihosting->cmdline != NULL ?
					semihosting->cmdline : "";
				uint32_t len = strlen(arg) + 1;
				if (len > size)
					semihosting->result = -1;
				else {
					semihosting_set_field(target, len, 1, fields);
					retval = target_write_buffer(target, addr, len,
							(uint8_t *)arg);
					if (retval != ERROR_OK)
						return retval;
					semihosting->result = 0;

					retval = semihosting_write_fields(target, 2, fields);
					if (retval != ERROR_OK)
						return retval;
				}
				LOG_DEBUG("SYS_GET_CMDLINE=[%s],%d", arg,
					(int)semihosting->result);
			}
			break;

		case SEMIHOSTING_SYS_HEAPINFO:	/* 0x16 */
			/*
			 * Returns the system stack and heap parameters.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains the address of a
			 * pointer to a four-field data block. The contents of the data
			 * block are filled by the function. The following C-like
			 * pseudocode describes the layout of the block:
			 * struct block {
			 *   void* heap_base;
			 *   void* heap_limit;
			 *   void* stack_base;
			 *   void* stack_limit;
			 * };
			 *
			 * Return
			 * On exit, the PARAMETER REGISTER is unchanged and the data
			 * block has been updated.
			 */
			retval = semihosting_read_fields(target, 1, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				uint64_t addr = semihosting_get_field(target, 0, fields);
				/* tell the remote we have no idea */
				memset(fields, 0, 4 * semihosting->word_size_bytes);
				retval = target_write_memory(target, addr, 4,
						semihosting->word_size_bytes,
						fields);
				if (retval != ERROR_OK)
					return retval;
				semihosting->result = 0;
			}
			break;

		case SEMIHOSTING_SYS_ISERROR:	/* 0x08 */
			/*
			 * Determines whether the return code from another semihosting
			 * call is an error status or not.
			 *
			 * This call is passed a parameter block containing the error
			 * code to examine.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * one-field data block:
			 * - field 1 The required status word to check.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - 0 if the status field is not an error indication
			 * - A nonzero value if the status field is an error indication.
			 */
			retval = semihosting_read_fields(target, 1, fields);
			if (retval != ERROR_OK)
				return retval;

			uint64_t code = semihosting_get_field(target, 0, fields);
			semihosting->result = (code != 0);
			break;

		case SEMIHOSTING_SYS_ISTTY:	/* 0x09 */
			/*
			 * Checks whether a file is connected to an interactive device.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * one-field argument block:
			 * field 1 A handle for a previously opened file object.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - 1 if the handle identifies an interactive device.
			 * - 0 if the handle identifies a file.
			 * - A value other than 1 or 0 if an error occurs.
			 */
			if (semihosting->is_fileio) {
				semihosting->hit_fileio = true;
				fileio_info->identifier = "isatty";
				fileio_info->param_1 = semihosting->param;
			} else {
				retval = semihosting_read_fields(target, 1, fields);
				if (retval != ERROR_OK)
					return retval;
				int fd = semihosting_get_field(target, 0, fields);
				semihosting->result = isatty(fd);
				LOG_DEBUG("isatty(%d)=%d", fd, (int)semihosting->result);
			}
			break;

		case SEMIHOSTING_SYS_OPEN:	/* 0x01 */
			/*
			 * Opens a file on the host system.
			 *
			 * The file path is specified either as relative to the current
			 * directory of the host process, or absolute, using the path
			 * conventions of the host operating system.
			 *
			 * Semihosting implementations must support opening the special
			 * path name :semihosting-features as part of the semihosting
			 * extensions reporting mechanism.
			 *
			 * ARM targets interpret the special path name :tt as meaning
			 * the console input stream, for an open-read or the console
			 * output stream, for an open-write. Opening these streams is
			 * performed as part of the standard startup code for those
			 * applications that reference the C stdio streams. The
			 * semihosting extension SH_EXT_STDOUT_STDERR allows the
			 * semihosting caller to open separate output streams
			 * corresponding to stdout and stderr. This extension is
			 * reported using feature byte 0, bit 1. Use SYS_OPEN with
			 * the special path name :semihosting-features to access the
			 * feature bits.
			 *
			 * If this extension is supported, the implementation must
			 * support the following additional semantics to SYS_OPEN:
			 * - If the special path name :tt is opened with an fopen
			 * mode requesting write access (w, wb, w+, or w+b), then
			 * this is a request to open stdout.
			 * - If the special path name :tt is opened with a mode
			 * requesting append access (a, ab, a+, or a+b), then this is
			 * a request to open stderr.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * three-field argument block:
			 * - field 1 A pointer to a null-terminated string containing
			 * a file or device name.
			 * - field 2 An integer that specifies the file opening mode.
			 * - field 3 An integer that gives the length of the string
			 * pointed to by field 1.
			 *
			 * The length does not include the terminating null character
			 * that must be present.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - A nonzero handle if the call is successful.
			 * - –1 if the call is not successful.
			 */
			retval = semihosting_read_fields(target, 3, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				uint64_t addr = semihosting_get_field(target, 0, fields);
				uint32_t mode = semihosting_get_field(target, 1, fields);
				size_t len = semihosting_get_field(target, 2, fields);

				if (mode > 11) {
					semihosting->result = -1;
					semihosting->sys_errno = EINVAL;
					break;
				}
				uint8_t *fn = malloc(len+1);
				if (!fn) {
					semihosting->result = -1;
					semihosting->sys_errno = ENOMEM;
				} else {
					retval = target_read_memory(target, addr, 1, len, fn);
					if (retval != ERROR_OK) {
						free(fn);
						return retval;
					}
					fn[len] = 0;
					/* TODO: implement the :semihosting-features special file.
					 * */
					if (semihosting->is_fileio) {
						if (strcmp((char *)fn, ":semihosting-features") == 0) {
							semihosting->result = -1;
							semihosting->sys_errno = EINVAL;
						} else if (strcmp((char *)fn, ":tt") == 0) {
							if (mode == 0)
								semihosting->result = 0;
							else if (mode == 4)
								semihosting->result = 1;
							else if (mode == 8)
								semihosting->result = 2;
							else
								semihosting->result = -1;
						} else {
							semihosting->hit_fileio = true;
							fileio_info->identifier = "open";
							fileio_info->param_1 = addr;
							fileio_info->param_2 = len;
							fileio_info->param_3 = open_modeflags[mode];
							fileio_info->param_4 = 0644;
						}
					} else {
						if (strcmp((char *)fn, ":tt") == 0) {
							/* Mode is:
							 * - 0-3 ("r") for stdin,
							 * - 4-7 ("w") for stdout,
							 * - 8-11 ("a") for stderr */
							if (mode < 4) {
								semihosting->result = dup(
										STDIN_FILENO);
								semihosting->sys_errno = errno;
								LOG_DEBUG("dup(STDIN)=%d",
									(int)semihosting->result);
							} else if (mode < 8) {
								semihosting->result = dup(
										STDOUT_FILENO);
								semihosting->sys_errno = errno;
								LOG_DEBUG("dup(STDOUT)=%d",
									(int)semihosting->result);
							} else {
								semihosting->result = dup(
										STDERR_FILENO);
								semihosting->sys_errno = errno;
								LOG_DEBUG("dup(STDERR)=%d",
									(int)semihosting->result);
							}
						} else {
							/* cygwin requires the permission setting
							 * otherwise it will fail to reopen a previously
							 * written file */
							semihosting->result = open((char *)fn,
									open_modeflags[mode],
									0644);
							semihosting->sys_errno = errno;
							LOG_DEBUG("open('%s')=%d", fn,
								(int)semihosting->result);
						}
					}
					free(fn);
				}
			}
			break;

		case SEMIHOSTING_SYS_READ:	/* 0x06 */
			/*
			 * Reads the contents of a file into a buffer. The file position
			 * is specified either:
			 * - Explicitly by a SYS_SEEK.
			 * - Implicitly one byte beyond the previous SYS_READ or
			 * SYS_WRITE request.
			 *
			 * The file position is at the start of the file when it is
			 * opened, and is lost when the file is closed. Perform the
			 * file operation as a single action whenever possible. For
			 * example, do not split a read of 16KB into four 4KB chunks
			 * unless there is no alternative.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * three-field data block:
			 * - field 1 Contains a handle for a file previously opened
			 * with SYS_OPEN.
			 * - field 2 Points to a buffer.
			 * - field 3 Contains the number of bytes to read to the buffer
			 * from the file.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains the number of bytes not
			 * filled in the buffer (buffer_length - bytes_read) as follows:
			 * - If the RETURN REGISTER is 0, the entire buffer was
			 * successfully filled.
			 * - If the RETURN REGISTER is the same as field 3, no bytes
			 * were read (EOF can be assumed).
			 * - If the RETURN REGISTER contains a value smaller than
			 * field 3, the read succeeded but the buffer was only partly
			 * filled. For interactive devices, this is the most common
			 * return value.
			 */
			retval = semihosting_read_fields(target, 3, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				int fd = semihosting_get_field(target, 0, fields);
				uint64_t addr = semihosting_get_field(target, 1, fields);
				size_t len = semihosting_get_field(target, 2, fields);
				if (semihosting->is_fileio) {
					semihosting->hit_fileio = true;
					fileio_info->identifier = "read";
					fileio_info->param_1 = fd;
					fileio_info->param_2 = addr;
					fileio_info->param_3 = len;
				} else {
					uint8_t *buf = malloc(len);
					if (!buf) {
						semihosting->result = -1;
						semihosting->sys_errno = ENOMEM;
					} else {
						semihosting->result = read(fd, buf, len);
						semihosting->sys_errno = errno;
						LOG_DEBUG("read(%d, 0x%" PRIx64 ", %zu)=%d",
							fd,
							addr,
							len,
							(int)semihosting->result);
						if (semihosting->result >= 0) {
							retval = target_write_buffer(target, addr,
									semihosting->result,
									buf);
							if (retval != ERROR_OK) {
								free(buf);
								return retval;
							}
							/* the number of bytes NOT filled in */
							semihosting->result = len -
								semihosting->result;
						}
						free(buf);
					}
				}
			}
			break;

		case SEMIHOSTING_SYS_READC:	/* 0x07 */
			/*
			 * Reads a byte from the console.
			 *
			 * Entry
			 * The PARAMETER REGISTER must contain 0. There are no other
			 * parameters or values possible.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains the byte read from
			 * the console.
			 */
			if (semihosting->is_fileio) {
				LOG_ERROR("SYS_READC not supported by semihosting fileio");
				return ERROR_FAIL;
			}
			semihosting->result = getchar();
			LOG_DEBUG("getchar()=%d", (int)semihosting->result);
			break;

		case SEMIHOSTING_SYS_REMOVE:	/* 0x0E */
			/*
			 * Deletes a specified file on the host filing system.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * two-field argument block:
			 * - field 1 Points to a null-terminated string that gives the
			 * path name of the file to be deleted.
			 * - field 2 The length of the string.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - 0 if the delete is successful
			 * - A nonzero, host-specific error code if the delete fails.
			 */
			retval = semihosting_read_fields(target, 2, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				uint64_t addr = semihosting_get_field(target, 0, fields);
				size_t len = semihosting_get_field(target, 1, fields);
				if (semihosting->is_fileio) {
					semihosting->hit_fileio = true;
					fileio_info->identifier = "unlink";
					fileio_info->param_1 = addr;
					fileio_info->param_2 = len;
				} else {
					uint8_t *fn = malloc(len+1);
					if (!fn) {
						semihosting->result = -1;
						semihosting->sys_errno = ENOMEM;
					} else {
						retval =
							target_read_memory(target, addr, 1, len,
								fn);
						if (retval != ERROR_OK) {
							free(fn);
							return retval;
						}
						fn[len] = 0;
						semihosting->result = remove((char *)fn);
						semihosting->sys_errno = errno;
						LOG_DEBUG("remove('%s')=%d", fn,
							(int)semihosting->result);

						free(fn);
					}
				}
			}
			break;

		case SEMIHOSTING_SYS_RENAME:	/* 0x0F */
			/*
			 * Renames a specified file.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * four-field data block:
			 * - field 1 A pointer to the name of the old file.
			 * - field 2 The length of the old filename.
			 * - field 3 A pointer to the new filename.
			 * - field 4 The length of the new filename. Both strings are
			 * null-terminated.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - 0 if the rename is successful.
			 * - A nonzero, host-specific error code if the rename fails.
			 */
			retval = semihosting_read_fields(target, 4, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				uint64_t addr1 = semihosting_get_field(target, 0, fields);
				size_t len1 = semihosting_get_field(target, 1, fields);
				uint64_t addr2 = semihosting_get_field(target, 2, fields);
				size_t len2 = semihosting_get_field(target, 3, fields);
				if (semihosting->is_fileio) {
					semihosting->hit_fileio = true;
					fileio_info->identifier = "rename";
					fileio_info->param_1 = addr1;
					fileio_info->param_2 = len1;
					fileio_info->param_3 = addr2;
					fileio_info->param_4 = len2;
				} else {
					uint8_t *fn1 = malloc(len1+1);
					uint8_t *fn2 = malloc(len2+1);
					if (!fn1 || !fn2) {
						free(fn1);
						free(fn2);
						semihosting->result = -1;
						semihosting->sys_errno = ENOMEM;
					} else {
						retval = target_read_memory(target, addr1, 1, len1,
								fn1);
						if (retval != ERROR_OK) {
							free(fn1);
							free(fn2);
							return retval;
						}
						retval = target_read_memory(target, addr2, 1, len2,
								fn2);
						if (retval != ERROR_OK) {
							free(fn1);
							free(fn2);
							return retval;
						}
						fn1[len1] = 0;
						fn2[len2] = 0;
						semihosting->result = rename((char *)fn1,
								(char *)fn2);
						semihosting->sys_errno = errno;
						LOG_DEBUG("rename('%s', '%s')=%d", fn1, fn2,
							(int)semihosting->result);

						free(fn1);
						free(fn2);
					}
				}
			}
			break;

		case SEMIHOSTING_SYS_SEEK:	/* 0x0A */
			/*
			 * Seeks to a specified position in a file using an offset
			 * specified from the start of the file. The file is assumed
			 * to be a byte array and the offset is given in bytes.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * two-field data block:
			 * - field 1 A handle for a seekable file object.
			 * - field 2 The absolute byte position to seek to.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - 0 if the request is successful.
			 * - A negative value if the request is not successful.
			 * Use SYS_ERRNO to read the value of the host errno variable
			 * describing the error.
			 *
			 * Note: The effect of seeking outside the current extent of
			 * the file object is undefined.
			 */
			retval = semihosting_read_fields(target, 2, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				int fd = semihosting_get_field(target, 0, fields);
				off_t pos = semihosting_get_field(target, 1, fields);
				if (semihosting->is_fileio) {
					semihosting->hit_fileio = true;
					fileio_info->identifier = "lseek";
					fileio_info->param_1 = fd;
					fileio_info->param_2 = pos;
					fileio_info->param_3 = SEEK_SET;
				} else {
					semihosting->result = lseek(fd, pos, SEEK_SET);
					semihosting->sys_errno = errno;
					LOG_DEBUG("lseek(%d, %d)=%d", fd, (int)pos,
						(int)semihosting->result);
					if (semihosting->result == pos)
						semihosting->result = 0;
				}
			}
			break;

		case SEMIHOSTING_SYS_SYSTEM:	/* 0x12 */
			/*
			 * Passes a command to the host command-line interpreter.
			 * This enables you to execute a system command such as dir,
			 * ls, or pwd. The terminal I/O is on the host, and is not
			 * visible to the target.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * two-field argument block:
			 * - field 1 Points to a string to be passed to the host
			 * command-line interpreter.
			 * - field 2 The length of the string.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains the return status.
			 */

			/* Provide SYS_SYSTEM functionality.  Uses the
			 * libc system command, there may be a reason *NOT*
			 * to use this, but as I can't think of one, I
			 * implemented it this way.
			 */
			retval = semihosting_read_fields(target, 2, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				uint64_t addr = semihosting_get_field(target, 0, fields);
				size_t len = semihosting_get_field(target, 1, fields);
				if (semihosting->is_fileio) {
					semihosting->hit_fileio = true;
					fileio_info->identifier = "system";
					fileio_info->param_1 = addr;
					fileio_info->param_2 = len;
				} else {
					uint8_t *cmd = malloc(len+1);
					if (!cmd) {
						semihosting->result = -1;
						semihosting->sys_errno = ENOMEM;
					} else {
						retval = target_read_memory(target,
								addr,
								1,
								len,
								cmd);
						if (retval != ERROR_OK) {
							free(cmd);
							return retval;
						} else {
							cmd[len] = 0;
							semihosting->result = system(
									(const char *)cmd);
							LOG_DEBUG("system('%s')=%d",
								cmd,
								(int)semihosting->result);
						}

						free(cmd);
					}
				}
			}
			break;

		case SEMIHOSTING_SYS_TIME:	/* 0x11 */
			/*
			 * Returns the number of seconds since 00:00 January 1, 1970.
			 * This value is real-world time, regardless of any debug agent
			 * configuration.
			 *
			 * Entry
			 * There are no parameters.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains the number of seconds.
			 */
			semihosting->result = time(NULL);
			break;

		case SEMIHOSTING_SYS_WRITE:	/* 0x05 */
			/*
			 * Writes the contents of a buffer to a specified file at the
			 * current file position. The file position is specified either:
			 * - Explicitly, by a SYS_SEEK.
			 * - Implicitly as one byte beyond the previous SYS_READ or
			 * SYS_WRITE request.
			 *
			 * The file position is at the start of the file when the file
			 * is opened, and is lost when the file is closed.
			 *
			 * Perform the file operation as a single action whenever
			 * possible. For example, do not split a write of 16KB into
			 * four 4KB chunks unless there is no alternative.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to a
			 * three-field data block:
			 * - field 1 Contains a handle for a file previously opened
			 * with SYS_OPEN.
			 * - field 2 Points to the memory containing the data to be written.
			 * - field 3 Contains the number of bytes to be written from
			 * the buffer to the file.
			 *
			 * Return
			 * On exit, the RETURN REGISTER contains:
			 * - 0 if the call is successful.
			 * - The number of bytes that are not written, if there is an error.
			 */
			retval = semihosting_read_fields(target, 3, fields);
			if (retval != ERROR_OK)
				return retval;
			else {
				int fd = semihosting_get_field(target, 0, fields);
				uint64_t addr = semihosting_get_field(target, 1, fields);
				size_t len = semihosting_get_field(target, 2, fields);
				if (semihosting->is_fileio) {
					semihosting->hit_fileio = true;
					fileio_info->identifier = "write";
					fileio_info->param_1 = fd;
					fileio_info->param_2 = addr;
					fileio_info->param_3 = len;
				} else {
					uint8_t *buf = malloc(len);
					if (!buf) {
						semihosting->result = -1;
						semihosting->sys_errno = ENOMEM;
					} else {
						retval = target_read_buffer(target, addr, len, buf);
						if (retval != ERROR_OK) {
							free(buf);
							return retval;
						}
						semihosting->result = write(fd, buf, len);
						semihosting->sys_errno = errno;
						LOG_DEBUG("write(%d, 0x%" PRIx64 ", %zu)=%d",
							fd,
							addr,
							len,
							(int)semihosting->result);
						if (semihosting->result >= 0) {
							/* The number of bytes that are NOT written.
							 * */
							semihosting->result = len -
								semihosting->result;
						}

						free(buf);
					}
				}
			}
			break;

		case SEMIHOSTING_SYS_WRITEC:	/* 0x03 */
			/*
			 * Writes a character byte, pointed to by the PARAMETER REGISTER,
			 * to the debug channel. When executed under a semihosting
			 * debugger, the character appears on the host debugger console.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to the
			 * character.
			 *
			 * Return
			 * None. The RETURN REGISTER is corrupted.
			 */
			if (semihosting->is_fileio) {
				semihosting->hit_fileio = true;
				fileio_info->identifier = "write";
				fileio_info->param_1 = 1;
				fileio_info->param_2 = semihosting->param;
				fileio_info->param_3 = 1;
			} else {
				uint64_t addr = semihosting->param;
				unsigned char c;
				retval = target_read_memory(target, addr, 1, 1, &c);
				if (retval != ERROR_OK)
					return retval;
				putchar(c);
				semihosting->result = 0;
			}
			break;

		case SEMIHOSTING_SYS_WRITE0:	/* 0x04 */
			/*
			 * Writes a null-terminated string to the debug channel.
			 * When executed under a semihosting debugger, the characters
			 * appear on the host debugger console.
			 *
			 * Entry
			 * On entry, the PARAMETER REGISTER contains a pointer to the
			 * first byte of the string.
			 *
			 * Return
			 * None. The RETURN REGISTER is corrupted.
			 */
			if (semihosting->is_fileio) {
				size_t count = 0;
				uint64_t addr = semihosting->param;
				for (;; addr++) {
					unsigned char c;
					retval = target_read_memory(target, addr, 1, 1, &c);
					if (retval != ERROR_OK)
						return retval;
					if (c == '\0')
						break;
					count++;
				}
				semihosting->hit_fileio = true;
				fileio_info->identifier = "write";
				fileio_info->param_1 = 1;
				fileio_info->param_2 = semihosting->param;
				fileio_info->param_3 = count;
			} else {
				uint64_t addr = semihosting->param;
				do {
					unsigned char c;
					retval = target_read_memory(target, addr++, 1, 1, &c);
					if (retval != ERROR_OK)
						return retval;
					if (!c)
						break;
					putchar(c);
				} while (1);
				semihosting->result = 0;
			}
			break;

		case SEMIHOSTING_SYS_ELAPSED:	/* 0x30 */
		/*
		 * Returns the number of elapsed target ticks since execution
		 * started.
		 * Use SYS_TICKFREQ to determine the tick frequency.
		 *
		 * Entry (32-bit)
		 * On entry, the PARAMETER REGISTER points to a two-field data
		 * block to be used for returning the number of elapsed ticks:
		 * - field 1 The least significant field and is at the low address.
		 * - field 2 The most significant field and is at the high address.
		 *
		 * Entry (64-bit)
		 * On entry the PARAMETER REGISTER points to a one-field data
		 * block to be used for returning the number of elapsed ticks:
		 * - field 1 The number of elapsed ticks as a 64-bit value.
		 *
		 * Return
		 * On exit:
		 * - On success, the RETURN REGISTER contains 0, the PARAMETER
		 * REGISTER is unchanged, and the data block pointed to by the
		 * PARAMETER REGISTER is filled in with the number of elapsed
		 * ticks.
		 * - On failure, the RETURN REGISTER contains -1, and the
		 * PARAMETER REGISTER contains -1.
		 *
		 * Note: Some semihosting implementations might not support this
		 * semihosting operation, and they always return -1 in the
		 * RETURN REGISTER.
		 */

		case SEMIHOSTING_SYS_TICKFREQ:	/* 0x31 */
		/*
		 * Returns the tick frequency.
		 *
		 * Entry
		 * The PARAMETER REGISTER must contain 0 on entry to this routine.
		 *
		 * Return
		 * On exit, the RETURN REGISTER contains either:
		 * - The number of ticks per second.
		 * - –1 if the target does not know the value of one tick.
		 *
		 * Note: Some semihosting implementations might not support
		 * this semihosting operation, and they always return -1 in the
		 * RETURN REGISTER.
		 */

		case SEMIHOSTING_SYS_TMPNAM:	/* 0x0D */
		/*
		 * Returns a temporary name for a file identified by a system
		 * file identifier.
		 *
		 * Entry
		 * On entry, the PARAMETER REGISTER contains a pointer to a
		 * three-word argument block:
		 * - field 1 A pointer to a buffer.
		 * - field 2 A target identifier for this filename. Its value
		 * must be an integer in the range 0-255.
		 * - field 3 Contains the length of the buffer. The length must
		 * be at least the value of L_tmpnam on the host system.
		 *
		 * Return
		 * On exit, the RETURN REGISTER contains:
		 * - 0 if the call is successful.
		 * - –1 if an error occurs.
		 *
		 * The buffer pointed to by the PARAMETER REGISTER contains
		 * the filename, prefixed with a suitable directory name.
		 * If you use the same target identifier again, the same
		 * filename is returned.
		 *
		 * Note: The returned string must be null-terminated.
		 */

		default:
			fprintf(stderr, "semihosting: unsupported call %#x\n",
				(unsigned) semihosting->op);
			semihosting->result = -1;
			semihosting->sys_errno = ENOTSUP;
	}

	if (!semihosting->hit_fileio) {
		retval = semihosting->post_result(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to post semihosting result");
			return retval;
		}
	}

	return ERROR_OK;
}

/* -------------------------------------------------------------------------
 * Local functions. */

static int semihosting_common_fileio_info(struct target *target,
	struct gdb_fileio_info *fileio_info)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting)
		return ERROR_FAIL;

	/*
	 * To avoid unnecessary duplication, semihosting prepares the
	 * fileio_info structure out-of-band when the target halts. See
	 * do_semihosting for more detail.
	 */
	if (!semihosting->is_fileio || !semihosting->hit_fileio)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int semihosting_common_fileio_end(struct target *target, int result,
	int fileio_errno, bool ctrl_c)
{
	struct gdb_fileio_info *fileio_info = target->fileio_info;
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting)
		return ERROR_FAIL;

	/* clear pending status */
	semihosting->hit_fileio = false;

	semihosting->result = result;
	semihosting->sys_errno = fileio_errno;

	/*
	 * Some fileio results do not match up with what the semihosting
	 * operation expects; for these operations, we munge the results
	 * below:
	 */
	switch (semihosting->op) {
		case SEMIHOSTING_SYS_WRITE:	/* 0x05 */
			if (result < 0)
				semihosting->result = fileio_info->param_3;
			else
				semihosting->result = 0;
			break;

		case SEMIHOSTING_SYS_READ:	/* 0x06 */
			if (result == (int)fileio_info->param_3)
				semihosting->result = 0;
			if (result <= 0)
				semihosting->result = fileio_info->param_3;
			break;

		case SEMIHOSTING_SYS_SEEK:	/* 0x0a */
			if (result > 0)
				semihosting->result = 0;
			break;
	}

	return semihosting->post_result(target);
}

/**
 * Read all fields of a command from target to buffer.
 */
static int semihosting_read_fields(struct target *target, size_t number,
	uint8_t *fields)
{
	struct semihosting *semihosting = target->semihosting;
	/* Use 4-byte multiples to trigger fast memory access. */
	return target_read_memory(target, semihosting->param, 4,
			number * (semihosting->word_size_bytes / 4), fields);
}

/**
 * Write all fields of a command from buffer to target.
 */
static int semihosting_write_fields(struct target *target, size_t number,
	uint8_t *fields)
{
	struct semihosting *semihosting = target->semihosting;
	/* Use 4-byte multiples to trigger fast memory access. */
	return target_write_memory(target, semihosting->param, 4,
			number * (semihosting->word_size_bytes / 4), fields);
}

/**
 * Extract a field from the buffer, considering register size and endianness.
 */
static uint64_t semihosting_get_field(struct target *target, size_t index,
	uint8_t *fields)
{
	struct semihosting *semihosting = target->semihosting;
	if (semihosting->word_size_bytes == 8)
		return target_buffer_get_u64(target, fields + (index * 8));
	else
		return target_buffer_get_u32(target, fields + (index * 4));
}

/**
 * Store a field in the buffer, considering register size and endianness.
 */
static void semihosting_set_field(struct target *target, uint64_t value,
	size_t index,
	uint8_t *fields)
{
	struct semihosting *semihosting = target->semihosting;
	if (semihosting->word_size_bytes == 8)
		target_buffer_set_u64(target, fields + (index * 8), value);
	else
		target_buffer_set_u32(target, fields + (index * 4), value);
}


/* -------------------------------------------------------------------------
 * Common semihosting commands handlers. */

static __COMMAND_HANDLER(handle_common_semihosting_command)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target == NULL) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		command_print(CMD, "semihosting not supported for current target");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0) {
		int is_active;

		COMMAND_PARSE_ENABLE(CMD_ARGV[0], is_active);

		if (!target_was_examined(target)) {
			LOG_ERROR("Target not examined yet");
			return ERROR_FAIL;
		}

		if (semihosting && semihosting->setup(target, is_active) != ERROR_OK) {
			LOG_ERROR("Failed to Configure semihosting");
			return ERROR_FAIL;
		}

		/* FIXME never let that "catch" be dropped! (???) */
		semihosting->is_active = is_active;
	}

	command_print(CMD, "semihosting is %s",
		semihosting->is_active
		? "enabled" : "disabled");

	return ERROR_OK;
}

static __COMMAND_HANDLER(handle_common_semihosting_fileio_command)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target == NULL) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		command_print(CMD, "semihosting not supported for current target");
		return ERROR_FAIL;
	}

	if (!semihosting->is_active) {
		command_print(CMD, "semihosting not yet enabled for current target");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0)
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], semihosting->is_fileio);

	command_print(CMD, "semihosting fileio is %s",
		semihosting->is_fileio
		? "enabled" : "disabled");

	return ERROR_OK;
}

static __COMMAND_HANDLER(handle_common_semihosting_cmdline)
{
	struct target *target = get_current_target(CMD_CTX);
	unsigned int i;

	if (target == NULL) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		command_print(CMD, "semihosting not supported for current target");
		return ERROR_FAIL;
	}

	free(semihosting->cmdline);
	semihosting->cmdline = CMD_ARGC > 0 ? strdup(CMD_ARGV[0]) : NULL;

	for (i = 1; i < CMD_ARGC; i++) {
		char *cmdline = alloc_printf("%s %s", semihosting->cmdline, CMD_ARGV[i]);
		if (cmdline == NULL)
			break;
		free(semihosting->cmdline);
		semihosting->cmdline = cmdline;
	}

	command_print(CMD, "semihosting command line is [%s]",
		semihosting->cmdline);

	return ERROR_OK;
}

static __COMMAND_HANDLER(handle_common_semihosting_resumable_exit_command)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target == NULL) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		command_print(CMD, "semihosting not supported for current target");
		return ERROR_FAIL;
	}

	if (!semihosting->is_active) {
		command_print(CMD, "semihosting not yet enabled for current target");
		return ERROR_FAIL;
	}

	if (CMD_ARGC > 0)
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], semihosting->has_resumable_exit);

	command_print(CMD, "semihosting resumable exit is %s",
		semihosting->has_resumable_exit
		? "enabled" : "disabled");

	return ERROR_OK;
}

const struct command_registration semihosting_common_handlers[] = {
	{
		"semihosting",
		.handler = handle_common_semihosting_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting operations",
	},
	{
		"semihosting_cmdline",
		.handler = handle_common_semihosting_cmdline,
		.mode = COMMAND_EXEC,
		.usage = "arguments",
		.help = "command line arguments to be passed to program",
	},
	{
		"semihosting_fileio",
		.handler = handle_common_semihosting_fileio_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting fileio operations",
	},
	{
		"semihosting_resexit",
		.handler = handle_common_semihosting_resumable_exit_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting resumable exit",
	},
	COMMAND_REGISTRATION_DONE
};
