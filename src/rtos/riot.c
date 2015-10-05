/***************************************************************************
 *   Copyright (C) 2015 by Daniel Krebs                                    *
 *   Daniel Krebs - github@daniel-krebs.net                                *
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

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include "target/target.h"
#include "target/target_type.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "target/armv7m.h"
#include "rtos_riot_stackings.h"

static bool riot_detect_rtos(struct target *target);
static int riot_create(struct target *target);
static int riot_update_threads(struct rtos *rtos);
static int riot_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
	struct rtos_reg **reg_list, int *num_regs);
static int riot_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);

struct riot_thread_state {
	int value;
	const char *desc;
};

/* refer RIOT sched.h */
static const struct riot_thread_state riot_thread_states[] = {
	{ 0, "Stopped" },
	{ 1, "Zombie" },
	{ 2, "Sleeping" },
	{ 3, "Blocked mutex" },
	{ 4, "Blocked receive" },
	{ 5, "Blocked send" },
	{ 6, "Blocked reply" },
	{ 7, "Blocked any flag" },
	{ 8, "Blocked all flags" },
	{ 9, "Blocked mbox" },
	{ 10, "Blocked condition" },
	{ 11, "Running" },
	{ 12, "Pending" },
};
#define RIOT_NUM_STATES ARRAY_SIZE(riot_thread_states)

struct riot_params {
	const char *target_name;
	unsigned char thread_sp_offset;
	unsigned char thread_status_offset;
};

static const struct riot_params riot_params_list[] = {
	{
		"cortex_m",		/* target_name */
		0x00,					/* thread_sp_offset */
		0x04,					/* thread_status_offset */
	},
	{	/* STLink */
		"hla_target",		/* target_name */
		0x00,			/* thread_sp_offset */
		0x04,			/* thread_status_offset */
	}
};
#define RIOT_NUM_PARAMS ARRAY_SIZE(riot_params_list)

/* Initialize in riot_create() depending on architecture */
static const struct rtos_register_stacking *stacking_info;

enum riot_symbol_values {
	RIOT_THREADS_BASE = 0,
	RIOT_NUM_THREADS,
	RIOT_ACTIVE_PID,
	RIOT_MAX_THREADS,
	RIOT_NAME_OFFSET,
};

/* refer RIOT core/sched.c */
static const char *const riot_symbol_list[] = {
	"sched_threads",
	"sched_num_threads",
	"sched_active_pid",
	"max_threads",
	"_tcb_name_offset",
	NULL
};

/* Define which symbols are not mandatory */
static const enum riot_symbol_values riot_optional_symbols[] = {
	RIOT_NAME_OFFSET,
};

const struct rtos_type riot_rtos = {
	.name = "RIOT",
	.detect_rtos = riot_detect_rtos,
	.create = riot_create,
	.update_threads = riot_update_threads,
	.get_thread_reg_list = riot_get_thread_reg_list,
	.get_symbol_list_to_lookup = riot_get_symbol_list_to_lookup,
};

static int riot_update_threads(struct rtos *rtos)
{
	int retval;
	unsigned int tasks_found = 0;
	const struct riot_params *param;

	if (rtos == NULL)
		return ERROR_FAIL;

	if (rtos->rtos_specific_params == NULL)
		return ERROR_FAIL;

	param = (const struct riot_params *)rtos->rtos_specific_params;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for RIOT");
		return ERROR_FAIL;
	}

	if (rtos->symbols[RIOT_THREADS_BASE].address == 0) {
		LOG_ERROR("Can't find symbol `%s`",
			riot_symbol_list[RIOT_THREADS_BASE]);
		return ERROR_FAIL;
	}

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* Reset values */
	rtos->current_thread = 0;
	rtos->thread_count = 0;

	/* read the current thread id */
	int16_t active_pid = 0;
	retval = target_read_u16(rtos->target,
			rtos->symbols[RIOT_ACTIVE_PID].address,
			(uint16_t *)&active_pid);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't read symbol `%s`",
			riot_symbol_list[RIOT_ACTIVE_PID]);
		return retval;
	}
	rtos->current_thread = active_pid;

	/* read the current thread count
	 * It's `int` in RIOT, but this is Cortex M* only anyway */
	int32_t thread_count = 0;
	retval = target_read_u16(rtos->target,
			rtos->symbols[RIOT_NUM_THREADS].address,
			(uint16_t *)&thread_count);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't read symbol `%s`",
			riot_symbol_list[RIOT_NUM_THREADS]);
		return retval;
	}
	rtos->thread_count = thread_count;

	/* read the maximum number of threads */
	uint8_t max_threads = 0;
	retval = target_read_u8(rtos->target,
			rtos->symbols[RIOT_MAX_THREADS].address,
			&max_threads);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't read symbol `%s`",
			riot_symbol_list[RIOT_MAX_THREADS]);
		return retval;
	}

	/* Base address of thread array */
	uint32_t threads_base = rtos->symbols[RIOT_THREADS_BASE].address;

	/* Try to get the offset of tcb_t::name, if absent RIOT wasn't compiled
	 * with DEVELHELP, so there are no thread names */
	uint8_t name_offset = 0;
	if (rtos->symbols[RIOT_NAME_OFFSET].address != 0) {
		retval = target_read_u8(rtos->target,
				rtos->symbols[RIOT_NAME_OFFSET].address,
				&name_offset);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't read symbol `%s`",
				riot_symbol_list[RIOT_NAME_OFFSET]);
			return retval;
		}
	}

	/* Allocate memory for thread description */
	rtos->thread_details = calloc(thread_count, sizeof(struct thread_detail));
	if (rtos->thread_details == NULL) {
		LOG_ERROR("RIOT: out of memory");
		return ERROR_FAIL;
	}

	/* Buffer for thread names, maximum to display is 32 */
	char buffer[32];

	for (unsigned int i = 0; i < max_threads; i++) {
		/* get pointer to tcb_t */
		uint32_t tcb_pointer = 0;
		retval = target_read_u32(rtos->target,
				threads_base + (i * 4),
				&tcb_pointer);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
			goto error;
		}

		if (tcb_pointer == 0) {
			/* PID unused */
			continue;
		}

		/* Index is PID */
		rtos->thread_details[tasks_found].threadid = i;

		/* read thread state */
		uint8_t status = 0;
		retval = target_read_u8(rtos->target,
				tcb_pointer + param->thread_status_offset,
				&status);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
			goto error;
		}

		/* Search for state */
		unsigned int k;
		for (k = 0; k < RIOT_NUM_STATES; k++) {
			if (riot_thread_states[k].value == status)
				break;
		}

		/* Copy state string */
		if (k >= RIOT_NUM_STATES) {
			rtos->thread_details[tasks_found].extra_info_str =
			strdup("unknown state");
		} else {
			rtos->thread_details[tasks_found].extra_info_str =
			strdup(riot_thread_states[k].desc);
		}

		if (rtos->thread_details[tasks_found].extra_info_str == NULL) {
			LOG_ERROR("RIOT: out of memory");
			retval = ERROR_FAIL;
			goto error;
		}

		/* Thread names are only available if compiled with DEVELHELP */
		if (name_offset != 0) {
			uint32_t name_pointer = 0;
			retval = target_read_u32(rtos->target,
					tcb_pointer + name_offset,
					&name_pointer);
			if (retval != ERROR_OK) {
				LOG_ERROR("Can't parse `%s`",
					riot_symbol_list[RIOT_THREADS_BASE]);
				goto error;
			}

			/* read thread name */
			retval = target_read_buffer(rtos->target,
					name_pointer,
					sizeof(buffer),
					(uint8_t *)&buffer);
			if (retval != ERROR_OK) {
				LOG_ERROR("Can't parse `%s`",
					riot_symbol_list[RIOT_THREADS_BASE]);
				goto error;
			}

			/* Make sure the string in the buffer terminates */
			if (buffer[sizeof(buffer) - 1] != 0)
				buffer[sizeof(buffer) - 1] = 0;

			/* Copy thread name */
			rtos->thread_details[tasks_found].thread_name_str =
			strdup(buffer);

		} else {
			rtos->thread_details[tasks_found].thread_name_str =
			strdup("Enable DEVELHELP to see task names");
		}

		if (rtos->thread_details[tasks_found].thread_name_str == NULL) {
			LOG_ERROR("RIOT: out of memory");
			retval = ERROR_FAIL;
			goto error;
		}

		rtos->thread_details[tasks_found].exists = true;

		tasks_found++;
	}

	return ERROR_OK;

error:
	rtos_free_threadlist(rtos);
	return retval;
}

static int riot_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct riot_params *param;

	if (rtos == NULL)
		return ERROR_FAIL;

	if (thread_id == 0)
		return ERROR_FAIL;

	if (rtos->rtos_specific_params == NULL)
		return ERROR_FAIL;

	param = (const struct riot_params *)rtos->rtos_specific_params;

	/* find the thread with given thread id */
	uint32_t threads_base = rtos->symbols[RIOT_THREADS_BASE].address;
	uint32_t tcb_pointer = 0;
	retval = target_read_u32(rtos->target,
			threads_base + (thread_id * 4),
			&tcb_pointer);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
		return retval;
	}

	/* read stack pointer for that thread */
	uint32_t stackptr = 0;
	retval = target_read_u32(rtos->target,
			tcb_pointer + param->thread_sp_offset,
			&stackptr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't parse `%s`", riot_symbol_list[RIOT_THREADS_BASE]);
		return retval;
	}

	return rtos_generic_stack_read(rtos->target,
			stacking_info,
			stackptr,
			reg_list,
			num_regs);
}

static int riot_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	*symbol_list = calloc(ARRAY_SIZE(riot_symbol_list), sizeof(symbol_table_elem_t));

	if (*symbol_list == NULL) {
		LOG_ERROR("RIOT: out of memory");
		return ERROR_FAIL;
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(riot_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = riot_symbol_list[i];
		(*symbol_list)[i].optional = false;

		/* Lookup if symbol is optional */
		for (unsigned int k = 0; k < sizeof(riot_optional_symbols); k++) {
			if (i == riot_optional_symbols[k]) {
				(*symbol_list)[i].optional = true;
				break;
			}
		}
	}

	return ERROR_OK;
}

static bool riot_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols != NULL) &&
		(target->rtos->symbols[RIOT_THREADS_BASE].address != 0)) {
		/* looks like RIOT */
		return true;
	}
	return false;
}

static int riot_create(struct target *target)
{
	unsigned int i = 0;

	/* lookup if target is supported by RIOT */
	while ((i < RIOT_NUM_PARAMS) &&
		(0 != strcmp(riot_params_list[i].target_name, target->type->name))) {
		i++;
	}
	if (i >= RIOT_NUM_PARAMS) {
		LOG_ERROR("Could not find target in RIOT compatibility list");
		return ERROR_FAIL;
	}

	target->rtos->rtos_specific_params = (void *)&riot_params_list[i];
	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;

	/* Stacking is different depending on architecture */
	struct armv7m_common *armv7m_target = target_to_armv7m(target);

	if (armv7m_target->arm.is_armv6m)
		stacking_info = &rtos_riot_cortex_m0_stacking;
	else if (is_armv7m(armv7m_target))
		stacking_info = &rtos_riot_cortex_m34_stacking;
	else {
		LOG_ERROR("No stacking info for architecture");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}
