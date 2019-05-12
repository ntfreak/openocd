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
#include "rtos_embkernel_stackings.h"

#define EMBKERNEL_MAX_THREAD_NAME_STR_SIZE (64)

static bool embKernel_detect_rtos(struct target *target);
static int embKernel_create(struct target *target);
static int embKernel_update_threads(struct rtos *rtos);
static int embKernel_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int embKernel_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);

struct rtos_type embKernel_rtos = {
		.name = "embKernel",
		.detect_rtos = embKernel_detect_rtos,
		.create = embKernel_create,
		.update_threads = embKernel_update_threads,
		.get_thread_reg_list =
		embKernel_get_thread_reg_list,
		.get_symbol_list_to_lookup = embKernel_get_symbol_list_to_lookup,
};

enum {
	SYMBOL_ID_sCurrentTask = 0,
	SYMBOL_ID_sListReady = 1,
	SYMBOL_ID_sListSleep = 2,
	SYMBOL_ID_sListSuspended = 3,
	SYMBOL_ID_sMaxPriorities = 4,
	SYMBOL_ID_sCurrentTaskCount = 5,
};

static const char * const embKernel_symbol_list[] = {
		"Rtos::sCurrentTask",
		"Rtos::sListReady",
		"Rtos::sListSleep",
		"Rtos::sListSuspended",
		"Rtos::sMaxPriorities",
		"Rtos::sCurrentTaskCount",
		NULL };

struct embKernel_params {
	const char *target_name;
	const unsigned char pointer_width;
	const unsigned char thread_count_width;
	const unsigned char rtos_list_size;
	const unsigned char thread_stack_offset;
	const unsigned char thread_name_offset;
	const unsigned char thread_priority_offset;
	const unsigned char thread_priority_width;
	const unsigned char iterable_next_offset;
	const unsigned char iterable_task_owner_offset;
	const struct rtos_register_stacking *stacking_info;
};

static const struct embKernel_params embKernel_params_list[] = {
		{
			"cortex_m", /* target_name */
			4, /* pointer_width */
			4, /* thread_count_width */
			8, /*rtos_list_size */
			0, /*thread_stack_offset */
			4, /*thread_name_offset */
			8, /*thread_priority_offset */
			4, /*thread_priority_width */
			4, /*iterable_next_offset */
			12, /*iterable_task_owner_offset */
			&rtos_embkernel_Cortex_M_stacking, /* stacking_info*/
		},
		{ "hla_target", /* target_name */
			4, /* pointer_width */
			4, /* thread_count_width */
			8, /*rtos_list_size */
			0, /*thread_stack_offset */
			4, /*thread_name_offset */
			8, /*thread_priority_offset */
			4, /*thread_priority_width */
			4, /*iterable_next_offset */
			12, /*iterable_task_owner_offset */
			&rtos_embkernel_Cortex_M_stacking, /* stacking_info */
		}
};

static bool embKernel_detect_rtos(struct target *target)
{
	if (target->rtos->symbols != NULL) {
		if (target->rtos->symbols[SYMBOL_ID_sCurrentTask].address != 0)
			return true;
	}
	return false;
}

static int embKernel_create(struct target *target)
{
	size_t i = 0;
	while ((i < ARRAY_SIZE(embKernel_params_list)) &&
			(0 != strcmp(embKernel_params_list[i].target_name, target->type->name)))
		i++;

	if (i >= ARRAY_SIZE(embKernel_params_list)) {
		LOG_WARNING("Could not find target \"%s\" in embKernel compatibility "
				"list", target->type->name);
		return -1;
	}

	target->rtos->rtos_specific_params = (void *) &embKernel_params_list[i];
	return 0;
}

static int embKernel_get_tasks_details(struct rtos *rtos, int64_t iterable, const struct embKernel_params *param,
		struct thread_detail *details, const char *state_str)
{
	int64_t task = 0;
	int retval = target_read_buffer(rtos->target, iterable + param->iterable_task_owner_offset, param->pointer_width,
			(uint8_t *) &task);
	if (retval != ERROR_OK)
		return retval;
	details->threadid = (threadid_t) task;
	details->exists = true;

	int64_t name_ptr = 0;
	retval = target_read_buffer(rtos->target, task + param->thread_name_offset, param->pointer_width,
			(uint8_t *) &name_ptr);
	if (retval != ERROR_OK)
		return retval;

	details->thread_name_str = malloc(EMBKERNEL_MAX_THREAD_NAME_STR_SIZE);
	if (name_ptr) {
		retval = target_read_buffer(rtos->target, name_ptr, EMBKERNEL_MAX_THREAD_NAME_STR_SIZE,
				(uint8_t *) details->thread_name_str);
		if (retval != ERROR_OK)
			return retval;
		details->thread_name_str[EMBKERNEL_MAX_THREAD_NAME_STR_SIZE - 1] = 0;
	} else {
		snprintf(details->thread_name_str, EMBKERNEL_MAX_THREAD_NAME_STR_SIZE, "NoName:[0x%08X]", (unsigned int) task);
	}

	int64_t priority = 0;
	retval = target_read_buffer(rtos->target, task + param->thread_priority_offset, param->thread_priority_width,
			(uint8_t *) &priority);
	if (retval != ERROR_OK)
		return retval;
	details->extra_info_str = malloc(EMBKERNEL_MAX_THREAD_NAME_STR_SIZE);
	if (task == rtos->current_thread) {
		snprintf(details->extra_info_str, EMBKERNEL_MAX_THREAD_NAME_STR_SIZE, "State: Running, Priority: %u",
				(unsigned int) priority);
	} else {
		snprintf(details->extra_info_str, EMBKERNEL_MAX_THREAD_NAME_STR_SIZE, "State: %s, Priority: %u",
				state_str, (unsigned int) priority);
	}

	LOG_OUTPUT("Getting task details: iterable=0x%08X, task=0x%08X, name=%s\n", (unsigned int)iterable,
			(unsigned int)task, details->thread_name_str);
	return 0;
}

static int embKernel_update_threads(struct rtos *rtos)
{
	/* int i = 0; */
	int retval;
	const struct embKernel_params *param;

	if (rtos == NULL)
		return -1;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for embKernel");
		return -4;
	}

	if (rtos->symbols[SYMBOL_ID_sCurrentTask].address == 0) {
		LOG_ERROR("Don't have the thread list head");
		return -2;
	}

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	param = (const struct embKernel_params *) rtos->rtos_specific_params;

	retval = target_read_buffer(rtos->target, rtos->symbols[SYMBOL_ID_sCurrentTask].address, param->pointer_width,
			(uint8_t *) &rtos->current_thread);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading current thread in embKernel thread list");
		return retval;
	}

	int64_t max_used_priority = 0;
	retval = target_read_buffer(rtos->target, rtos->symbols[SYMBOL_ID_sMaxPriorities].address, param->pointer_width,
			(uint8_t *) &max_used_priority);
	if (retval != ERROR_OK)
		return retval;

	int thread_list_size = 0;
	retval = target_read_buffer(rtos->target, rtos->symbols[SYMBOL_ID_sCurrentTaskCount].address,
			param->thread_count_width, (uint8_t *) &thread_list_size);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read embKernel thread count from target");
		return retval;
	}

	/* create space for new thread details */
	rtos->thread_details = malloc(sizeof(struct thread_detail) * thread_list_size);
	if (!rtos->thread_details) {
		LOG_ERROR("Error allocating memory for %d threads", thread_list_size);
		return ERROR_FAIL;
	}

	int threadIdx = 0;
	/* Look for ready tasks */
	for (int pri = 0; pri < max_used_priority; pri++) {
		/* Get first item in queue */
		int64_t iterable = 0;
		retval = target_read_buffer(rtos->target,
				rtos->symbols[SYMBOL_ID_sListReady].address + (pri * param->rtos_list_size), param->pointer_width,
				(uint8_t *) &iterable);
		if (retval != ERROR_OK)
			return retval;
		for (; iterable && threadIdx < thread_list_size; threadIdx++) {
			/* Get info from this iterable item */
			retval = embKernel_get_tasks_details(rtos, iterable, param, &rtos->thread_details[threadIdx], "Ready");
			if (retval != ERROR_OK)
				return retval;
			/* Get next iterable item */
			retval = target_read_buffer(rtos->target, iterable + param->iterable_next_offset, param->pointer_width,
					(uint8_t *) &iterable);
			if (retval != ERROR_OK)
				return retval;
		}
	}
	/* Look for sleeping tasks */
	int64_t iterable = 0;
	retval = target_read_buffer(rtos->target, rtos->symbols[SYMBOL_ID_sListSleep].address, param->pointer_width,
			(uint8_t *) &iterable);
	if (retval != ERROR_OK)
		return retval;
	for (; iterable && threadIdx < thread_list_size; threadIdx++) {
		/*Get info from this iterable item */
		retval = embKernel_get_tasks_details(rtos, iterable, param, &rtos->thread_details[threadIdx], "Sleeping");
		if (retval != ERROR_OK)
			return retval;
		/*Get next iterable item */
		retval = target_read_buffer(rtos->target, iterable + param->iterable_next_offset, param->pointer_width,
				(uint8_t *) &iterable);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Look for suspended tasks  */
	iterable = 0;
	retval = target_read_buffer(rtos->target, rtos->symbols[SYMBOL_ID_sListSuspended].address, param->pointer_width,
			(uint8_t *) &iterable);
	if (retval != ERROR_OK)
		return retval;
	for (; iterable && threadIdx < thread_list_size; threadIdx++) {
		/* Get info from this iterable item */
		retval = embKernel_get_tasks_details(rtos, iterable, param, &rtos->thread_details[threadIdx], "Suspended");
		if (retval != ERROR_OK)
			return retval;
		/*Get next iterable item */
		retval = target_read_buffer(rtos->target, iterable + param->iterable_next_offset, param->pointer_width,
				(uint8_t *) &iterable);
		if (retval != ERROR_OK)
			return retval;
	}

	rtos->thread_count = 0;
	rtos->thread_count = threadIdx;
	LOG_OUTPUT("Found %u tasks\n", (unsigned int)threadIdx);
	return 0;
}

static int embKernel_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct embKernel_params *param;
	int64_t stack_ptr = 0;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -1;

	param = (const struct embKernel_params *) rtos->rtos_specific_params;

	/* Read the stack pointer */
	retval = target_read_buffer(rtos->target, thread_id + param->thread_stack_offset, param->pointer_width,
			(uint8_t *) &stack_ptr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading stack frame from embKernel thread");
		return retval;
	}

	return rtos_generic_stack_read(rtos->target, param->stacking_info, stack_ptr, reg_list, num_regs);
}

static int embKernel_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(ARRAY_SIZE(embKernel_symbol_list), sizeof(symbol_table_elem_t));

	for (i = 0; i < ARRAY_SIZE(embKernel_symbol_list); i++)
		(*symbol_list)[i].symbol_name = embKernel_symbol_list[i];

	return 0;
}
