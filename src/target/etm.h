/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007 by Vincent Palatin                                 *
 *   vincent.palatin_openocd@m4x.org                                       *
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

#ifndef OPENOCD_TARGET_ETM_H
#define OPENOCD_TARGET_ETM_H

#include "trace.h"
#include "arm_jtag.h"

struct image;

/* ETM registers (JTAG protocol) */
enum {
	ETM_CTRL = 0x00,
	ETM_CONFIG = 0x01,
	ETM_TRIG_EVENT = 0x02,
	ETM_ASIC_CTRL = 0x03,
	ETM_STATUS = 0x04,
	ETM_SYS_CONFIG = 0x05,
	ETM_TRACE_RESOURCE_CTRL = 0x06,
	ETM_TRACE_EN_CTRL2 = 0x07,
	ETM_TRACE_EN_EVENT = 0x08,
	ETM_TRACE_EN_CTRL1 = 0x09,
	/* optional FIFOFULL */
	ETM_FIFOFULL_REGION = 0x0a,
	ETM_FIFOFULL_LEVEL = 0x0b,
	/* viewdata support */
	ETM_VIEWDATA_EVENT = 0x0c,
	ETM_VIEWDATA_CTRL1 = 0x0d,
	ETM_VIEWDATA_CTRL2 = 0x0e,	/* optional */
	ETM_VIEWDATA_CTRL3 = 0x0f,
	/* N pairs of ADDR_{COMPARATOR,ACCESS} registers */
	ETM_ADDR_COMPARATOR_VALUE = 0x10,
	ETM_ADDR_ACCESS_TYPE = 0x20,
	/* N pairs of DATA_COMPARATOR_{VALUE,MASK} registers */
	ETM_DATA_COMPARATOR_VALUE = 0x30,
	ETM_DATA_COMPARATOR_MASK = 0x40,
	/* N quads of COUNTER_{RELOAD_{VALUE,EVENT},ENABLE,VALUE} registers */
	ETM_COUNTER_RELOAD_VALUE = 0x50,
	ETM_COUNTER_ENABLE = 0x54,
	ETM_COUNTER_RELOAD_EVENT = 0x58,
	ETM_COUNTER_VALUE = 0x5c,
	/* 6 sequencer event transitions */
	ETM_SEQUENCER_EVENT = 0x60,
	ETM_SEQUENCER_STATE = 0x67,
	/* N triggered outputs */
	ETM_EXTERNAL_OUTPUT = 0x68,
	/* N task contexts */
	ETM_CONTEXTID_COMPARATOR_VALUE = 0x6c,
	ETM_CONTEXTID_COMPARATOR_MASK = 0x6f,
	ETM_ID = 0x79,
};

struct etm_reg {
	uint8_t value[4];
	const struct etm_reg_info *reg_info;
	struct arm_jtag *jtag_info;
};

/* Subset of ETM_CTRL bit assignments.  Many of these
 * control the configuration of trace output, which
 * hooks up either to ETB or to an external device.
 *
 * NOTE that these have evolved since the ~v1.3 defns ...
 */
enum {
	ETM_CTRL_POWERDOWN	= (1 << 0),
	ETM_CTRL_MONITOR_CPRT	= (1 << 1),

	/* bits 3:2 == trace type */
	ETM_CTRL_TRACE_DATA	= (1 << 2),
	ETM_CTRL_TRACE_ADDR	= (2 << 2),
	ETM_CTRL_TRACE_MASK	= (3 << 2),

	/* Port width (bits 21 and 6:4) */
	ETM_PORT_4BIT		= 0x00,
	ETM_PORT_8BIT		= 0x10,
	ETM_PORT_16BIT		= 0x20,
	ETM_PORT_24BIT		= 0x30,
	ETM_PORT_32BIT		= 0x40,
	ETM_PORT_48BIT		= 0x50,
	ETM_PORT_64BIT		= 0x60,
	ETM_PORT_1BIT		= 0x00 | (1 << 21),
	ETM_PORT_2BIT		= 0x10 | (1 << 21),
	ETM_PORT_WIDTH_MASK	= 0x70 | (1 << 21),

	ETM_CTRL_FIFOFULL_STALL	= (1 << 7),
	ETM_CTRL_BRANCH_OUTPUT	= (1 << 8),
	ETM_CTRL_DBGRQ		= (1 << 9),
	ETM_CTRL_ETM_PROG	= (1 << 10),
	ETM_CTRL_ETMEN		= (1 << 11),
	ETM_CTRL_CYCLE_ACCURATE	= (1 << 12),

	/* Clocking modes -- up to v2.1, bit 13 */
	ETM_PORT_FULL_CLOCK	= (0 << 13),
	ETM_PORT_HALF_CLOCK	= (1 << 13),
	ETM_PORT_CLOCK_MASK	= (1 << 13),

	/* bits 15:14 == context ID size used in tracing */
	ETM_CTRL_CONTEXTID_NONE	= (0 << 14),
	ETM_CTRL_CONTEXTID_8	= (1 << 14),
	ETM_CTRL_CONTEXTID_16	= (2 << 14),
	ETM_CTRL_CONTEXTID_32	= (3 << 14),
	ETM_CTRL_CONTEXTID_MASK	= (3 << 14),

	/* Port modes -- bits 17:16, tied to clocking mode */
	ETM_PORT_NORMAL		= (0 << 16),
	ETM_PORT_MUXED		= (1 << 16),
	ETM_PORT_DEMUXED	= (2 << 16),
	ETM_PORT_MODE_MASK	= (3 << 16),

	/* bits 31:18 defined in v3.0 and later (e.g. ARM11+) */
};

/* forward-declare ETM context */
struct etm_context;

struct etm_capture_driver {
	const char *name;
	const struct command_registration *commands;
	int (*init)(struct etm_context *etm_ctx);
	trace_status_t (*status)(struct etm_context *etm_ctx);
	int (*read_trace)(struct etm_context *etm_ctx);
	int (*start_capture)(struct etm_context *etm_ctx);
	int (*stop_capture)(struct etm_context *etm_ctx);
};

enum {
	ETMV1_TRACESYNC_CYCLE = 0x1,
	ETMV1_TRIGGER_CYCLE = 0x2,
};

struct etmv1_trace_data {
	uint8_t pipestat;	/* bits 0-2 pipeline status */
	uint16_t packet;		/* packet data (4, 8 or 16 bit) */
	int flags;		/* ETMV1_TRACESYNC_CYCLE, ETMV1_TRIGGER_CYCLE */
};

/* describe a trace context
 * if support for ETMv2 or ETMv3 is to be implemented,
 * this will have to be split into version independent elements
 * and a version specific part
 */
struct etm_context {
	struct target *target;		/* target this ETM is connected to */
	struct reg_cache *reg_cache;		/* ETM register cache */
	struct etm_capture_driver *capture_driver;	/* driver used to access ETM data */
	void *capture_driver_priv;	/* capture driver private data */
	trace_status_t capture_status;	/* current state of capture run */
	struct etmv1_trace_data *trace_data;	/* trace data */
	uint32_t trace_depth;		/* number of cycles to be analyzed, 0 if no data available */
	uint32_t control;	/* shadow of ETM_CTRL */
	int /*arm_state*/ core_state;	/* current core state */
	struct image *image;		/* source for target opcodes */
	uint32_t pipe_index;		/* current trace cycle */
	uint32_t data_index;		/* cycle holding next data packet */
	bool data_half;			/* port half on a 16 bit port */
	bool pc_ok;			/* full PC has been acquired */
	bool ptr_ok;			/* whether last_ptr is valid */
	uint8_t bcd_vers;		/* e.g. 0x13 == ETMv1.3 */
	uint32_t config;		/* cache of ETM_CONFIG value */
	uint32_t id;			/* cache of ETM_ID value, or 0 */
	uint32_t current_pc;		/* current program counter */
	uint32_t last_branch;		/* last branch address output */
	uint32_t last_branch_reason;	/* type of last branch encountered */
	uint32_t last_ptr;		/* address of the last data access */
	uint32_t last_instruction;	/* index of last executed (to calc timings) */
};

/* PIPESTAT values */
typedef enum {
	STAT_IE = 0x0,
	STAT_ID = 0x1,
	STAT_IN = 0x2,
	STAT_WT = 0x3,
	STAT_BE = 0x4,
	STAT_BD = 0x5,
	STAT_TR = 0x6,
	STAT_TD = 0x7
} etmv1_pipestat_t;

/* branch reason values */
typedef enum {
	BR_NORMAL  = 0x0, /* Normal PC change : periodic synchro (ETMv1.1) */
	BR_ENABLE  = 0x1, /* Trace has been enabled */
	BR_RESTART = 0x2, /* Trace restarted after a FIFO overflow */
	BR_NODEBUG = 0x3, /* ARM has exited for debug state */
	BR_PERIOD  = 0x4, /* Periodic synchronization point (ETM >= v1.2)*/
	BR_RSVD5   = 0x5, /* reserved */
	BR_RSVD6   = 0x6, /* reserved */
	BR_RSVD7   = 0x7, /* reserved */
} etmv1_branch_reason_t;

struct reg_cache *etm_build_reg_cache(struct target *target,
		struct arm_jtag *jtag_info, struct etm_context *etm_ctx);

int etm_setup(struct target *target);

extern const struct command_registration etm_command_handlers[];

#define ERROR_ETM_INVALID_DRIVER	(-1300)
#define ERROR_ETM_PORTMODE_NOT_SUPPORTED	(-1301)
#define ERROR_ETM_CAPTURE_INIT_FAILED	(-1302)
#define ERROR_ETM_ANALYSIS_FAILED	(-1303)

#endif /* OPENOCD_TARGET_ETM_H */
