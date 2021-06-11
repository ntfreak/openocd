/*
 * Copyright (C) 2010 by David Brownell
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Simple utility to parse and dump ARM Cortex-M3 SWO trace output.  Once the
 * mechanisms work right, this information can be used for various purposes
 * including profiling (particularly easy for flat PC-sample profiles) and
 * for debugging.
 *
 * SWO is the Single Wire Output found on some ARM cores, most notably on the
 * Cortex-M3.  It combines data from several sources:
 *
 *  - Software trace (ITM):  so-called "printf-style" application messaging
 *    using "ITM stimulus ports"; and differential timestamps.
 *  - Hardware trace (DWT):  for profiling counters and comparator matches.
 *  - TPIU may issue sync packets.
 *
 * The trace data format is defined in Appendix E, "Debug ITM and DWT packet
 * protocol", of the ARMv7-M Architecture Reference Manual (DDI 0403C).  It
 * is a superset of the ITM data format from the Coresight TRM.
 *
 * The trace data has two encodings.  The working assumption is that data
 * gets into this program using the UART encoding.
 */

#include <errno.h>
#include <libgen.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

unsigned int dump_swit;

/* Example ITM trace word (0xWWXXYYZZ) parsing for task events, sent
 * on port 31 (Reserved for "the" RTOS in CMSIS v1.30)
 *   WWXX: event code (0..3 pre-assigned, 4..15 reserved)
 *   YY:   task priority
 *   ZZ:   task number
 *
 * NOTE that this specific encoding could be space-optimized; and that
 * trace data streams could also be history-sensitive.
 */
static void show_task(int port, unsigned data)
{
	unsigned code = data >> 16;
	char buf[16];

	if (dump_swit)
		return;

	switch (code) {
	case 0:
		strcpy(buf, "run");
		break;
	case 1:
		strcpy(buf, "block");
		break;
	case 2:
		strcpy(buf, "create");
		break;
	case 3:
		strcpy(buf, "destroy");
		break;
	/* 4..15 reserved for other infrastructure ops */
	default:
		sprintf(buf, "code %d", code);
		break;
	}
	printf("TASK %d, pri %d: %s",
		(data >> 0) & 0xff,
		(data >> 8) & 0xff,
		buf);
}

static void show_reserved(FILE *f, char *label, int c)
{
	unsigned i;

	if (dump_swit)
		return;

	printf("%s - %#02x", label, c);

	for (i = 0; (c & 0x80) && i < 4; i++) {
		c = fgetc(f);
		if (c == EOF) {
			printf("(ERROR %d - %s) ", errno, strerror(errno));
			break;
		}
		printf(" %#02x", c);
	}

	printf("\n");
}

static bool read_varlen(FILE *f, int c, unsigned *value)
{
	unsigned size;
	unsigned char buf[4];

	*value = 0;

	switch (c & 3) {
	case 3:
		size = 4;
		break;
	case 2:
		size = 2;
		break;
	case 1:
		size = 1;
		break;
	default:
		printf("INVALID SIZE\n");
		return false;
	}

	memset(buf, 0, sizeof buf);
	if (fread(buf, 1, size, f) != size)
		goto err;

	*value =  (buf[3] << 24)
		+ (buf[2] << 16)
		+ (buf[1] << 8)
		+ (buf[0] << 0);
	return true;

err:
	printf("(ERROR %d - %s)\n", errno, strerror(errno));
	return false;
}

static void show_hard(FILE *f, int c)
{
	unsigned type = c >> 3;
	unsigned value;
	char *label;

	if (dump_swit)
		return;

	printf("DWT - ");

	if (!read_varlen(f, c, &value))
		return;
	printf("%#x", value);

	switch (type) {
	case 0:				/* event counter wrapping */
		printf("overflow %s%s%s%s%s%s",
			(value & (1 << 5)) ? "cyc " : "",
			(value & (1 << 4)) ? "fold " : "",
			(value & (1 << 3)) ? "lsu " : "",
			(value & (1 << 2)) ? "slp " : "",
			(value & (1 << 1)) ? "exc " : "",
			(value & (1 << 0)) ? "cpi " : "");
		break;
	case 1:				/* exception tracing */
		switch (value >> 12) {
		case 1:
			label = "entry to";
			break;
		case 2:
			label = "exit from";
			break;
		case 3:
			label = "return to";
			break;
		default:
			label = "?";
			break;
		}
		printf("%s exception %d", label, value & 0x1ff);
		break;
	case 2:				/* PC sampling */
		if (c == 0x15)
			printf("PC - sleep");
		else
			printf("PC - %#08x", value);
		break;
	case 8:				/* data tracing, pc value */
	case 10:
	case 12:
	case 14:
		printf("Data trace %d, PC %#08x", (c >> 4) & 3, value);
		/* optionally followed by data value */
		break;
	case 9:				/* data tracing, address offset */
	case 11:
	case 13:
	case 15:
		printf("Data trace %d, address offset %#04x",
				(c >> 4) & 3, value);
		/* always followed by data value */
		break;
	case 16 ... 23:			/* data tracing, data value */
		printf("Data trace %d, ", (c >> 4) & 3);
		label = (c & 0x8) ? "write" : "read";
		switch (c & 3) {
		case 3:
			printf("word %s, value %#08x", label, value);
			break;
		case 2:
			printf("halfword %s, value %#04x", label, value);
			break;
		case 1:
			printf("byte %s, value %#02x", label, value);
			break;
		}
		break;
	default:
		printf("UNDEFINED, rawtype: %x", type);
		break;
	}

	printf("\n");
	return;
}

/*
 * Table of SWIT (SoftWare InstrumentTation) message dump formats, for
 * ITM port 0..31 application data.
 *
 * Eventually this should be customizable; all usage is application defined.
 *
 * REVISIT there can be up to 256 trace ports, via "ITM Extension" packets
 */
struct {
	int port;
	void (*show)(int port, unsigned data);
} format[] = {
	{ .port = 31,  .show = show_task, },
};

static void show_swit(FILE *f, int c)
{
	unsigned port = c >> 3;
	unsigned value = 0;
	unsigned i;

	if (port + 1 == dump_swit) {
		if (!read_varlen(f, c, &value))
			return;
		printf("%c", value);
		return;
	}

	if (!read_varlen(f, c, &value))
		return;

	if (dump_swit)
		return;

	printf("SWIT %u - ", port);

	printf("%#08x", value);

	for (i = 0; i < sizeof(format) / sizeof(format[0]); i++) {
		if (format[i].port == port) {
			printf(", ");
			format[i].show(port, value);
			break;
		}
	}

	printf("\n");
	return;
}

static void show_timestamp(FILE *f, int c)
{
	unsigned counter = 0;
	char *label = "";
	bool delayed = false;

	if (dump_swit)
		return;

	printf("TIMESTAMP - ");

	/* Format 2: header only */
	if (!(c & 0x80)) {
		switch (c) {
		case 0:		/* sync packet -- coding error! */
		case 0x70:	/* overflow -- ditto! */
			printf("ERROR - %#02x\n", c);
			break;
		default:
			/* synchronous to ITM */
			counter = c >> 4;
			goto done;
		}
		return;
	}

	/* Format 1:  one to four bytes of data too */
	switch (c >> 4) {
	default:
		label = ", reserved control\n";
		break;
	case 0xc:
		/* synchronous to ITM */
		break;
	case 0xd:
		label = ", timestamp delayed";
		delayed = true;
		break;
	case 0xe:
		label = ", packet delayed";
		delayed = true;
		break;
	case 0xf:
		label = ", packet and timestamp delayed";
		delayed = true;
		break;
	}

	c = fgetc(f);
	if (c == EOF)
		goto err;
	counter = c & 0x7f;
	if (!(c & 0x80))
		goto done;

	c = fgetc(f);
	if (c == EOF)
		goto err;
	counter |= (c & 0x7f) << 7;
	if (!(c & 0x80))
		goto done;

	c = fgetc(f);
	if (c == EOF)
		goto err;
	counter |= (c & 0x7f) << 14;
	if (!(c & 0x80))
		goto done;

	c = fgetc(f);
	if (c == EOF)
		goto err;
	counter |= (c & 0x7f) << 21;

done:
	/* REVISIT should we try to convert from delta values?  */
	printf("+%u%s\n", counter, label);
	return;

err:
	printf("(ERROR %d - %s) ", errno, strerror(errno));
	goto done;
}

int main(int argc, char **argv)
{
	FILE *f = stdin;
	int c;

	/* parse arguments */
	while ((c = getopt(argc, argv, "f:d:")) != EOF) {
		switch (c) {
		case 'f':
			/* e.g. from UART connected to /dev/ttyUSB0 */
			f = fopen(optarg, "r");
			if (!f) {
				perror(optarg);
				return 1;
			}
			break;
		case 'd':
			dump_swit = atoi(optarg);
			break;
		default:
			fprintf(stderr, "usage: %s [-f input]",
				basename(argv[0]));
			return 1;
		}
	}

	/* Parse data ... records have a header then data bytes.
	 * NOTE: we assume getc() deals in 8-bit bytes.
	 */
	bool overflow = false;

	while ((c = getc(f)) != EOF) {

		/* Sync packet ... 7 zeroes, 0x80 */
		if (c == 0) {
			int i;

			for (i = 0; i < 6; i++) {
				c = fgetc(f);
				if (c == EOF)
					break;
				if (c != 0)
					goto bad_sync;
			}
			c = fgetc(f);
			if (c == 0x80) {
				printf("SYNC\n");
				continue;
			}
bad_sync:
			printf("BAD SYNC\n");
			continue;
		}

		/* Overflow packet */
		if (c == 0x70) {
			/* REVISIT later, report just what overflowed!
			 * Timestamp and SWIT can happen.  Non-ITM too?
			 */
			overflow = true;
			printf("OVERFLOW ...\n");
			continue;
		}
		overflow = false;

		switch (c & 0x0f) {
		case 0x00:		/* Timestamp */
			show_timestamp(f, c);
			break;
		case 0x04:		/* "Reserved" */
			show_reserved(f, "RESERVED", c);
			break;
		case 0x08:		/* ITM Extension */
			/* FIXME someday, handle these ...  */
			show_reserved(f, "ITM EXT", c);
			break;
		case 0x0c:		/* DWT Extension */
			show_reserved(f, "DWT EXT", c);
			break;
		default:
			if (c & 4)
				show_hard(f, c);
			else
				show_swit(f, c);
			break;
		}

	}

	return 0;
}
