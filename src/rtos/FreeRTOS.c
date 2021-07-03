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
#include "rtos_standard_stackings.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"

#define FREERTOS_MAX_PRIORITIES	63

/* FIXME: none of the _width parameters are actually observed properly!
 * you WILL need to edit more if you actually attempt to target a 8/16/64
 * bit target!
 */

struct freertos_params {
	const char *target_name;
	const unsigned char thread_count_width;
	const unsigned char pointer_width;
	const unsigned char list_next_offset;
	const unsigned char list_width;
	const unsigned char list_elem_next_offset;
	const unsigned char list_elem_content_offset;
	const unsigned char thread_stack_offset;
	const unsigned char thread_name_offset;
	const struct rtos_register_stacking *stacking_info_cm3;
	const struct rtos_register_stacking *stacking_info_cm4f;
	const struct rtos_register_stacking *stacking_info_cm4f_fpu;
};

static const struct freertos_params freertos_params_list[] = {
	{
	"cortex_m",			/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	16,						/* list_next_offset; */
	20,						/* list_width; */
	8,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	&rtos_standard_cortex_m3_stacking,	/* stacking_info */
	&rtos_standard_cortex_m4f_stacking,
	&rtos_standard_cortex_m4f_fpu_stacking,
	},
	{
	"hla_target",			/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	16,						/* list_next_offset; */
	20,						/* list_width; */
	8,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	&rtos_standard_cortex_m3_stacking,	/* stacking_info */
	&rtos_standard_cortex_m4f_stacking,
	&rtos_standard_cortex_m4f_fpu_stacking,
	},
	{
	"nds32_v3",			/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	16,						/* list_next_offset; */
	20,						/* list_width; */
	8,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	&rtos_standard_nds32_n1068_stacking,	/* stacking_info */
	&rtos_standard_cortex_m4f_stacking,
	&rtos_standard_cortex_m4f_fpu_stacking,
	},
};

static bool freertos_detect_rtos(struct target *target);
static int freertos_create(struct target *target);
static int freertos_update_threads(struct rtos *rtos);
static int freertos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int freertos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[]);

struct rtos_type freertos_rtos = {
	.name = "FreeRTOS",

	.detect_rtos = freertos_detect_rtos,
	.create = freertos_create,
	.update_threads = freertos_update_threads,
	.get_thread_reg_list = freertos_get_thread_reg_list,
	.get_symbol_list_to_lookup = freertos_get_symbol_list_to_lookup,
};

enum freertos_symbol_values {
	FREERTOS_VAL_PX_CURRENT_TCB = 0,
	FREERTOS_VAL_PX_READY_TASKS_LISTS = 1,
	FREERTOS_VAL_X_DELAYED_TASK_LIST1 = 2,
	FREERTOS_VAL_X_DELAYED_TASK_LIST2 = 3,
	FREERTOS_VAL_PX_DELAYED_TASK_LIST = 4,
	FREERTOS_VAL_PX_OVERFLOW_DELAYED_TASK_LIST = 5,
	FREERTOS_VAL_X_PENDING_READY_LIST = 6,
	FREERTOS_VAL_X_TASKS_WAITING_TERMINATION = 7,
	FREERTOS_VAL_X_SUSPENDED_TASK_LIST = 8,
	FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS = 9,
	FREERTOS_VAL_UX_TOP_USED_PRIORITY = 10,
};

struct symbols {
	const char *name;
	bool optional;
};

static const struct symbols freertos_symbol_list[] = {
	{ "pxCurrentTCB", false },
	{ "pxReadyTasksLists", false },
	{ "xDelayedTaskList1", false },
	{ "xDelayedTaskList2", false },
	{ "pxDelayedTaskList", false },
	{ "pxOverflowDelayedTaskList", false },
	{ "xPendingReadyList", false },
	{ "xTasksWaitingTermination", true }, /* Only if INCLUDE_vTaskDelete */
	{ "xSuspendedTaskList", true }, /* Only if INCLUDE_vTaskSuspend */
	{ "uxCurrentNumberOfTasks", false },
	{ "uxTopUsedPriority", true }, /* Unavailable since v7.5.3 */
	{ NULL, false }
};

/* TODO: */
/* this is not safe for little endian yet */
/* may be problems reading if sizes are not 32 bit long integers. */
/* test mallocs for failure */

static int freertos_update_threads(struct rtos *rtos)
{
	int retval;
	unsigned int tasks_found = 0;
	const struct freertos_params *param;

	if (!rtos->rtos_specific_params)
		return -1;

	param = (const struct freertos_params *) rtos->rtos_specific_params;

	if (!rtos->symbols) {
		LOG_ERROR("No symbols for FreeRTOS");
		return -3;
	}

	if (rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address == 0) {
		LOG_ERROR("Don't have the number of threads in FreeRTOS");
		return -2;
	}

	uint32_t thread_list_size = 0;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
			&thread_list_size);
	LOG_DEBUG("FreeRTOS: Read uxCurrentNumberOfTasks at 0x%" PRIx64 ", value %" PRIu32,
										rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
										thread_list_size);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS thread count from target");
		return retval;
	}

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* read the current thread */
	uint32_t pointer_casts_are_bad;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address,
			&pointer_casts_are_bad);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading current thread in FreeRTOS thread list");
		return retval;
	}
	rtos->current_thread = pointer_casts_are_bad;
	LOG_DEBUG("FreeRTOS: Read pxCurrentTCB at 0x%" PRIx64 ", value 0x%" PRIx64,
										rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address,
										rtos->current_thread);

	if ((thread_list_size  == 0) || (rtos->current_thread == 0)) {
		/* Either : No RTOS threads - there is always at least the current execution though */
		/* OR     : No current thread - all threads suspended - show the current execution
		 * of idling */
		char tmp_str[] = "Current Execution";
		thread_list_size++;
		tasks_found++;
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * thread_list_size);
		if (!rtos->thread_details) {
			LOG_ERROR("Error allocating memory for %d threads", thread_list_size);
			return ERROR_FAIL;
		}
		rtos->thread_details->threadid = 1;
		rtos->thread_details->exists = true;
		rtos->thread_details->extra_info_str = NULL;
		rtos->thread_details->thread_name_str = malloc(sizeof(tmp_str));
		strcpy(rtos->thread_details->thread_name_str, tmp_str);

		if (thread_list_size == 1) {
			rtos->thread_count = 1;
			return ERROR_OK;
		}
	} else {
		/* create space for new thread details */
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * thread_list_size);
		if (!rtos->thread_details) {
			LOG_ERROR("Error allocating memory for %d threads", thread_list_size);
			return ERROR_FAIL;
		}
	}

	/* Find out how many lists are needed to be read from pxReadyTasksLists, */
	if (rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address == 0) {
		LOG_ERROR("FreeRTOS: uxTopUsedPriority is not defined, consult the OpenOCD manual for a work-around");
		return ERROR_FAIL;
	}
	uint32_t top_used_priority = 0;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address,
			&top_used_priority);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("FreeRTOS: Read uxTopUsedPriority at 0x%" PRIx64 ", value %" PRIu32,
										rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address,
										top_used_priority);
	if (top_used_priority > FREERTOS_MAX_PRIORITIES) {
		LOG_ERROR("FreeRTOS top used priority is unreasonably big, not proceeding: %" PRIu32,
			top_used_priority);
		return ERROR_FAIL;
	}

	/* uxTopUsedPriority was defined as configMAX_PRIORITIES - 1
	 * in old FreeRTOS versions (before V7.5.3)
	 * Use contrib/rtos-helpers/FreeRTOS-openocd.c to get compatible symbol
	 * in newer FreeRTOS versions.
	 * Here we restore the original configMAX_PRIORITIES value */
	unsigned int config_max_priorities = top_used_priority + 1;

	symbol_address_t *list_of_lists =
		malloc(sizeof(symbol_address_t) * (config_max_priorities + 5));
	if (!list_of_lists) {
		LOG_ERROR("Error allocating memory for %u priorities", config_max_priorities);
		return ERROR_FAIL;
	}

	unsigned int num_lists;
	for (num_lists = 0; num_lists < config_max_priorities; num_lists++)
		list_of_lists[num_lists] = rtos->symbols[FREERTOS_VAL_PX_READY_TASKS_LISTS].address +
			num_lists * param->list_width;

	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_DELAYED_TASK_LIST1].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_DELAYED_TASK_LIST2].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_PENDING_READY_LIST].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_SUSPENDED_TASK_LIST].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_TASKS_WAITING_TERMINATION].address;

	for (unsigned int i = 0; i < num_lists; i++) {
		if (list_of_lists[i] == 0)
			continue;

		/* Read the number of threads in this list */
		uint32_t list_thread_count = 0;
		retval = target_read_u32(rtos->target,
				list_of_lists[i],
				&list_thread_count);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading number of threads in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read thread count for list %u at 0x%" PRIx64 ", value %" PRIu32,
										i, list_of_lists[i], list_thread_count);

		if (list_thread_count == 0)
			continue;

		/* Read the location of first list item */
		uint32_t prev_list_elem_ptr = -1;
		uint32_t list_elem_ptr = 0;
		retval = target_read_u32(rtos->target,
				list_of_lists[i] + param->list_next_offset,
				&list_elem_ptr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read first item for list %u at 0x%" PRIx64 ", value 0x%" PRIx32,
										i, list_of_lists[i] + param->list_next_offset, list_elem_ptr);

		while ((list_thread_count > 0) && (list_elem_ptr != 0) &&
				(list_elem_ptr != prev_list_elem_ptr) &&
				(tasks_found < thread_list_size)) {
			/* Get the location of the thread structure. */
			rtos->thread_details[tasks_found].threadid = 0;
			retval = target_read_u32(rtos->target,
					list_elem_ptr + param->list_elem_content_offset,
					&pointer_casts_are_bad);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading thread list item object in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			rtos->thread_details[tasks_found].threadid = pointer_casts_are_bad;
			LOG_DEBUG("FreeRTOS: Read Thread ID at 0x%" PRIx32 ", value 0x%" PRIx64,
										list_elem_ptr + param->list_elem_content_offset,
										rtos->thread_details[tasks_found].threadid);

			/* get thread name */

			#define FREERTOS_THREAD_NAME_STR_SIZE (200)
			char tmp_str[FREERTOS_THREAD_NAME_STR_SIZE];

			/* Read the thread name */
			retval = target_read_buffer(rtos->target,
					rtos->thread_details[tasks_found].threadid + param->thread_name_offset,
					FREERTOS_THREAD_NAME_STR_SIZE,
					(uint8_t *)&tmp_str);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			tmp_str[FREERTOS_THREAD_NAME_STR_SIZE-1] = '\x00';
			LOG_DEBUG("FreeRTOS: Read Thread Name at 0x%" PRIx64 ", value '%s'",
										rtos->thread_details[tasks_found].threadid + param->thread_name_offset,
										tmp_str);

			if (tmp_str[0] == '\x00')
				strcpy(tmp_str, "No Name");

			rtos->thread_details[tasks_found].thread_name_str =
				malloc(strlen(tmp_str)+1);
			strcpy(rtos->thread_details[tasks_found].thread_name_str, tmp_str);
			rtos->thread_details[tasks_found].exists = true;

			if (rtos->thread_details[tasks_found].threadid == rtos->current_thread) {
				char running_str[] = "State: Running";
				rtos->thread_details[tasks_found].extra_info_str = malloc(
						sizeof(running_str));
				strcpy(rtos->thread_details[tasks_found].extra_info_str,
					running_str);
			} else
				rtos->thread_details[tasks_found].extra_info_str = NULL;

			tasks_found++;
			list_thread_count--;

			prev_list_elem_ptr = list_elem_ptr;
			list_elem_ptr = 0;
			retval = target_read_u32(rtos->target,
					prev_list_elem_ptr + param->list_elem_next_offset,
					&list_elem_ptr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading next thread item location in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			LOG_DEBUG("FreeRTOS: Read next thread location at 0x%" PRIx32 ", value 0x%" PRIx32,
										prev_list_elem_ptr + param->list_elem_next_offset,
										list_elem_ptr);
		}
	}

	free(list_of_lists);
	rtos->thread_count = tasks_found;
	return 0;
}

static int freertos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct freertos_params *param;
	int64_t stack_ptr = 0;

	if (!rtos)
		return -1;

	if (thread_id == 0)
		return -2;

	if (!rtos->rtos_specific_params)
		return -1;

	param = (const struct freertos_params *) rtos->rtos_specific_params;

	/* Read the stack pointer */
	uint32_t pointer_casts_are_bad;
	retval = target_read_u32(rtos->target,
			thread_id + param->thread_stack_offset,
			&pointer_casts_are_bad);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading stack frame from FreeRTOS thread");
		return retval;
	}
	stack_ptr = pointer_casts_are_bad;
	LOG_DEBUG("FreeRTOS: Read stack pointer at 0x%" PRIx64 ", value 0x%" PRIx64,
										thread_id + param->thread_stack_offset,
										stack_ptr);

	/* Check for armv7m with *enabled* FPU, i.e. a Cortex-M4F */
	int cm4_fpu_enabled = 0;
	struct armv7m_common *armv7m_target = target_to_armv7m(rtos->target);
	if (is_armv7m(armv7m_target)) {
		if (armv7m_target->fp_feature == FPV4_SP) {
			/* Found ARM v7m target which includes a FPU */
			uint32_t cpacr;

			retval = target_read_u32(rtos->target, FPU_CPACR, &cpacr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Could not read CPACR register to check FPU state");
				return -1;
			}

			/* Check if CP10 and CP11 are set to full access. */
			if (cpacr & 0x00F00000) {
				/* Found target with enabled FPU */
				cm4_fpu_enabled = 1;
			}
		}
	}

	if (cm4_fpu_enabled == 1) {
		/* Read the LR to decide between stacking with or without FPU */
		uint32_t lr_svc = 0;
		retval = target_read_u32(rtos->target,
				stack_ptr + 0x20,
				&lr_svc);
		if (retval != ERROR_OK) {
			LOG_OUTPUT("Error reading stack frame from FreeRTOS thread");
			return retval;
		}
		if ((lr_svc & 0x10) == 0)
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f_fpu, stack_ptr, reg_list, num_regs);
		else
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f, stack_ptr, reg_list, num_regs);
	} else
		return rtos_generic_stack_read(rtos->target, param->stacking_info_cm3, stack_ptr, reg_list, num_regs);
}

static int freertos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(freertos_symbol_list), sizeof(struct symbol_table_elem));

	for (i = 0; i < ARRAY_SIZE(freertos_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = freertos_symbol_list[i].name;
		(*symbol_list)[i].optional = freertos_symbol_list[i].optional;
	}

	return 0;
}

#if 0

static int freertos_set_current_thread(struct rtos *rtos, threadid_t thread_id)
{
	return 0;
}

static int freertos_get_thread_ascii_info(struct rtos *rtos, threadid_t thread_id, char **info)
{
	int retval;
	const struct freertos_params *param;

	if (!rtos)
		return -1;

	if (thread_id == 0)
		return -2;

	if (!rtos->rtos_specific_params)
		return -3;

	param = (const struct freertos_params *) rtos->rtos_specific_params;

#define FREERTOS_THREAD_NAME_STR_SIZE (200)
	char tmp_str[FREERTOS_THREAD_NAME_STR_SIZE];

	/* Read the thread name */
	retval = target_read_buffer(rtos->target,
			thread_id + param->thread_name_offset,
			FREERTOS_THREAD_NAME_STR_SIZE,
			(uint8_t *)&tmp_str);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
		return retval;
	}
	tmp_str[FREERTOS_THREAD_NAME_STR_SIZE-1] = '\x00';

	if (tmp_str[0] == '\x00')
		strcpy(tmp_str, "No Name");

	*info = malloc(strlen(tmp_str)+1);
	strcpy(*info, tmp_str);
	return 0;
}

#endif

static bool freertos_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols) &&
			(target->rtos->symbols[FREERTOS_VAL_PX_READY_TASKS_LISTS].address != 0)) {
		/* looks like FreeRTOS */
		return true;
	}
	return false;
}

static int freertos_create(struct target *target)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(freertos_params_list); i++)
		if (strcmp(freertos_params_list[i].target_name, target->type->name) == 0) {
			target->rtos->rtos_specific_params = (void *)&freertos_params_list[i];
			return 0;
		}

	LOG_ERROR("Could not find target in FreeRTOS compatibility list");
	return -1;
}
