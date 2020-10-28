/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#ifndef OPENOCD_RTOS_RTOS_H
#define OPENOCD_RTOS_RTOS_H

#include "server/server.h"
#include "target/target.h"
#include <jim-nvp.h>

typedef int64_t threadid_t;
typedef int64_t symbol_address_t;

struct reg;

/**
 * Table should be terminated by an element with NULL in symbol_name
 */
typedef struct symbol_table_elem_struct {
	const char *symbol_name;
	symbol_address_t address;
	bool optional;
} symbol_table_elem_t;

struct thread_detail {
	threadid_t threadid;
	bool exists;
	char *thread_name_str;
	char *extra_info_str;
};

struct rtos {
	const struct rtos_type *type;

	symbol_table_elem_t *symbols;
	struct target *target;
	/*  add a context variable instead of global variable */
	/* The thread currently selected by gdb. */
	int64_t current_threadid;
	/* The currently selected thread according to the target. */
	threadid_t current_thread;
	struct thread_detail *thread_details;
	int thread_count;
	int (*gdb_thread_packet)(struct connection *connection, char const *packet, int packet_size);
	int (*gdb_target_for_threadid)(struct connection *connection, int64_t thread_id, struct target **p_target);
	void *rtos_specific_params;
};

struct rtos_reg {
	uint32_t number;
	uint32_t size;
	uint8_t value[16];
};

struct rtos_type {
	const char *name;
	bool (*detect_rtos)(struct target *target);
	int (*create)(struct target *target);
	int (*smp_init)(struct target *target);
	int (*update_threads)(struct rtos *rtos);
	/** Return a list of general registers, with their values filled out. */
	int (*get_thread_reg_list)(struct rtos *rtos, int64_t thread_id,
			struct rtos_reg **reg_list, int *num_regs);
	int (*get_thread_reg)(struct rtos *rtos, int64_t thread_id,
			uint32_t reg_num, struct rtos_reg *reg);
	int (*get_symbol_list_to_lookup)(symbol_table_elem_t *symbol_list[]);
	int (*clean)(struct target *target);
	char * (*ps_command)(struct target *target);
	int (*set_reg)(struct rtos *rtos, uint32_t reg_num, uint8_t *reg_value);
};

struct stack_register_offset {
	unsigned short number;		/* register number */
	signed short offset;		/* offset in bytes from stack head, or -1 to indicate
					 * register is not stacked, or -2 to indicate this is the
					 * stack pointer register */
	unsigned short width_bits;
};

struct rtos_register_stacking {
	unsigned char stack_registers_size;
	signed char stack_growth_direction;
	unsigned char num_output_registers;
	/* Some targets require evaluating the stack to determine the
	 * actual stack pointer for a process.  If this field is NULL,
	 * just use stacking->stack_registers_size * stack_growth_direction
	 * to calculate adjustment.
	 */
	int64_t (*calculate_process_stack)(struct target *target,
		const uint8_t *stack_data,
		const struct rtos_register_stacking *stacking,
		int64_t stack_ptr);
	const struct stack_register_offset *register_offsets;
};

#define GDB_THREAD_PACKET_NOT_CONSUMED (-40)

int rtos_create(Jim_GetOptInfo *goi, struct target *target);
void rtos_destroy(struct target *target);
int rtos_set_reg(struct connection *connection, int reg_num,
		uint8_t *reg_value);
int rtos_generic_stack_read(struct target *target,
		const struct rtos_register_stacking *stacking,
		int64_t stack_ptr,
		struct rtos_reg **reg_list,
		int *num_regs);
int gdb_thread_packet(struct connection *connection, char const *packet, int packet_size);
int rtos_get_gdb_reg(struct connection *connection, int reg_num);
int rtos_get_gdb_reg_list(struct connection *connection);
int rtos_update_threads(struct target *target);
void rtos_free_threadlist(struct rtos *rtos);
int rtos_smp_init(struct target *target);
/*  function for handling symbol access */
int rtos_qsymbol(struct connection *connection, char const *packet, int packet_size);

#endif /* OPENOCD_RTOS_RTOS_H */
