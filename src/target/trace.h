/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
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

#ifndef OPENOCD_TARGET_TRACE_H
#define OPENOCD_TARGET_TRACE_H

#include "helper/types.h"

struct target;
struct command_context;

struct trace_point {
	uint32_t address;
	uint64_t hit_counter;
};

struct trace {
	uint32_t num_trace_points;
	uint32_t trace_points_size;
	struct trace_point *trace_points;
	uint32_t trace_history_size;
	uint32_t *trace_history;
	uint32_t trace_history_pos;
	int trace_history_overflowed;
};

/**
 * \todo This enum is one of the few things in this file related
 * to *hardware* tracing ... split such "real" tracing out from
 * the contrib/libdcc support.
 */
typedef enum trace_status {
	TRACE_IDLE = 0x0,
	TRACE_RUNNING = 0x1,
	TRACE_TRIGGERED = 0x2,
	TRACE_COMPLETED = 0x4,
	TRACE_OVERFLOWED = 0x8,
} trace_status_t;

int trace_point(struct target *target, uint32_t number);
int trace_register_commands(struct command_context *cmd_ctx);

#define ERROR_TRACE_IMAGE_UNAVAILABLE		(-1500)
#define ERROR_TRACE_INSTRUCTION_UNAVAILABLE	(-1501)

#endif /* OPENOCD_TARGET_TRACE_H */
