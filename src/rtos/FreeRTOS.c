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

#define FreeRTOS_STRUCT(int_type, ptr_type, list_prev_offset)

struct FreeRTOS_params {
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

static const struct FreeRTOS_params FreeRTOS_params_list[] = {
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
	&rtos_standard_Cortex_M3_stacking,	/* stacking_info */
	&rtos_standard_Cortex_M4F_stacking,
	&rtos_standard_Cortex_M4F_FPU_stacking,
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
	&rtos_standard_Cortex_M3_stacking,	/* stacking_info */
	&rtos_standard_Cortex_M4F_stacking,
	&rtos_standard_Cortex_M4F_FPU_stacking,
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
	&rtos_standard_NDS32_N1068_stacking,	/* stacking_info */
	&rtos_standard_Cortex_M4F_stacking,
	&rtos_standard_Cortex_M4F_FPU_stacking,
	},
};

#define FREERTOS_NUM_PARAMS ((int)(sizeof(FreeRTOS_params_list)/sizeof(struct FreeRTOS_params)))

static bool FreeRTOS_detect_rtos(struct target *target);
static int FreeRTOS_create(struct target *target);
static int FreeRTOS_update_threads(struct rtos *rtos);
static int FreeRTOS_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int FreeRTOS_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);

struct rtos_type FreeRTOS_rtos = {
	.name = "FreeRTOS",

	.detect_rtos = FreeRTOS_detect_rtos,
	.create = FreeRTOS_create,
	.update_threads = FreeRTOS_update_threads,
	.get_thread_reg_list = FreeRTOS_get_thread_reg_list,
	.get_symbol_list_to_lookup = FreeRTOS_get_symbol_list_to_lookup,
};

enum FreeRTOS_symbol_values {
	FreeRTOS_VAL_pxCurrentTCB = 0,
	FreeRTOS_VAL_pxReadyTasksLists = 1,
	FreeRTOS_VAL_xDelayedTaskList1 = 2,
	FreeRTOS_VAL_xDelayedTaskList2 = 3,
	FreeRTOS_VAL_pxDelayedTaskList = 4,
	FreeRTOS_VAL_pxOverflowDelayedTaskList = 5,
	FreeRTOS_VAL_xPendingReadyList = 6,
	FreeRTOS_VAL_xTasksWaitingTermination = 7,
	FreeRTOS_VAL_xSuspendedTaskList = 8,
	FreeRTOS_VAL_uxCurrentNumberOfTasks = 9,
	FreeRTOS_VAL_uxTopUsedPriority = 10,
};

struct symbols {
	const char *name;
	bool optional;
};

static const struct symbols FreeRTOS_symbol_list[] = {
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

static int FreeRTOS_update_threads(struct rtos *rtos)
{
	int retval;
	int tasks_found = 0;
	const struct FreeRTOS_params *param;

	if (rtos->rtos_specific_params == NULL)
		return -1;

	param = (const struct FreeRTOS_params *) rtos->rtos_specific_params;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for FreeRTOS");
		return -3;
	}

	if (rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address == 0) {
		LOG_ERROR("Don't have the number of threads in FreeRTOS");
		return -2;
	}

	int thread_list_size = 0;
	retval = target_read_buffer(rtos->target,
			rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address,
			param->thread_count_width,
			(uint8_t *)&thread_list_size);
	LOG_DEBUG("FreeRTOS: Read uxCurrentNumberOfTasks at 0x%" PRIx64 ", value %d\r\n",
										rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address,
										thread_list_size);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS thread count from target");
		return retval;
	}

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* read the current thread */
	retval = target_read_buffer(rtos->target,
			rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address,
			param->pointer_width,
			(uint8_t *)&rtos->current_thread);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading current thread in FreeRTOS thread list");
		return retval;
	}
	LOG_DEBUG("FreeRTOS: Read pxCurrentTCB at 0x%" PRIx64 ", value 0x%" PRIx64 "\r\n",
										rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address,
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
	if (rtos->symbols[FreeRTOS_VAL_uxTopUsedPriority].address == 0) {
		LOG_ERROR("FreeRTOS: uxTopUsedPriority is not defined, consult the OpenOCD manual for a work-around");
		return ERROR_FAIL;
	}
	uint64_t top_used_priority = 0;
	/* FIXME: endianess error on almost all target_read_buffer(), see also
	 * other rtoses */
	retval = target_read_buffer(rtos->target,
			rtos->symbols[FreeRTOS_VAL_uxTopUsedPriority].address,
			param->pointer_width,
			(uint8_t *)&top_used_priority);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("FreeRTOS: Read uxTopUsedPriority at 0x%" PRIx64 ", value %" PRIu64 "\r\n",
										rtos->symbols[FreeRTOS_VAL_uxTopUsedPriority].address,
										top_used_priority);
	if (top_used_priority > FREERTOS_MAX_PRIORITIES) {
		LOG_ERROR("FreeRTOS top used priority is unreasonably big, not proceeding: %" PRIu64,
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
		list_of_lists[num_lists] = rtos->symbols[FreeRTOS_VAL_pxReadyTasksLists].address +
			num_lists * param->list_width;

	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xDelayedTaskList1].address;
	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xDelayedTaskList2].address;
	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xPendingReadyList].address;
	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xSuspendedTaskList].address;
	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xTasksWaitingTermination].address;

	for (unsigned int i = 0; i < num_lists; i++) {
		if (list_of_lists[i] == 0)
			continue;

		/* Read the number of threads in this list */
		int64_t list_thread_count = 0;
		retval = target_read_buffer(rtos->target,
				list_of_lists[i],
				param->thread_count_width,
				(uint8_t *)&list_thread_count);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading number of threads in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read thread count for list %u at 0x%" PRIx64 ", value %" PRId64 "\r\n",
										i, list_of_lists[i], list_thread_count);

		if (list_thread_count == 0)
			continue;

		/* Read the location of first list item */
		uint64_t prev_list_elem_ptr = -1;
		uint64_t list_elem_ptr = 0;
		retval = target_read_buffer(rtos->target,
				list_of_lists[i] + param->list_next_offset,
				param->pointer_width,
				(uint8_t *)&list_elem_ptr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read first item for list %u at 0x%" PRIx64 ", value 0x%" PRIx64 "\r\n",
										i, list_of_lists[i] + param->list_next_offset, list_elem_ptr);

		while ((list_thread_count > 0) && (list_elem_ptr != 0) &&
				(list_elem_ptr != prev_list_elem_ptr) &&
				(tasks_found < thread_list_size)) {
			/* Get the location of the thread structure. */
			rtos->thread_details[tasks_found].threadid = 0;
			retval = target_read_buffer(rtos->target,
					list_elem_ptr + param->list_elem_content_offset,
					param->pointer_width,
					(uint8_t *)&(rtos->thread_details[tasks_found].threadid));
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading thread list item object in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			LOG_DEBUG("FreeRTOS: Read Thread ID at 0x%" PRIx64 ", value 0x%" PRIx64 "\r\n",
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
			LOG_DEBUG("FreeRTOS: Read Thread Name at 0x%" PRIx64 ", value \"%s\"\r\n",
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
			retval = target_read_buffer(rtos->target,
					prev_list_elem_ptr + param->list_elem_next_offset,
					param->pointer_width,
					(uint8_t *)&list_elem_ptr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading next thread item location in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			LOG_DEBUG("FreeRTOS: Read next thread location at 0x%" PRIx64 ", value 0x%" PRIx64 "\r\n",
										prev_list_elem_ptr + param->list_elem_next_offset,
										list_elem_ptr);
		}
	}

	free(list_of_lists);
	rtos->thread_count = tasks_found;
	return 0;
}

static int FreeRTOS_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct FreeRTOS_params *param;
	int64_t stack_ptr = 0;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -1;

	param = (const struct FreeRTOS_params *) rtos->rtos_specific_params;

	/* Read the stack pointer */
	retval = target_read_buffer(rtos->target,
			thread_id + param->thread_stack_offset,
			param->pointer_width,
			(uint8_t *)&stack_ptr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading stack frame from FreeRTOS thread");
		return retval;
	}
	LOG_DEBUG("FreeRTOS: Read stack pointer at 0x%" PRIx64 ", value 0x%" PRIx64 "\r\n",
										thread_id + param->thread_stack_offset,
										stack_ptr);

	/* Check for armv7m with *enabled* FPU, i.e. a Cortex-M4F */
	int cm4_fpu_enabled = 0;
	struct armv7m_common *armv7m_target = target_to_armv7m(rtos->target);
	if (is_armv7m(armv7m_target)) {
		if (armv7m_target->fp_feature == FPv4_SP) {
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
		uint32_t LR_svc = 0;
		retval = target_read_buffer(rtos->target,
				stack_ptr + 0x20,
				param->pointer_width,
				(uint8_t *)&LR_svc);
		if (retval != ERROR_OK) {
			LOG_OUTPUT("Error reading stack frame from FreeRTOS thread\r\n");
			return retval;
		}
		if ((LR_svc & 0x10) == 0)
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f_fpu, stack_ptr, reg_list, num_regs);
		else
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f, stack_ptr, reg_list, num_regs);
	} else
		return rtos_generic_stack_read(rtos->target, param->stacking_info_cm3, stack_ptr, reg_list, num_regs);
}

static int FreeRTOS_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(FreeRTOS_symbol_list), sizeof(symbol_table_elem_t));

	for (i = 0; i < ARRAY_SIZE(FreeRTOS_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = FreeRTOS_symbol_list[i].name;
		(*symbol_list)[i].optional = FreeRTOS_symbol_list[i].optional;
	}

	return 0;
}

#if 0

static int FreeRTOS_set_current_thread(struct rtos *rtos, threadid_t thread_id)
{
	return 0;
}

static int FreeRTOS_get_thread_ascii_info(struct rtos *rtos, threadid_t thread_id, char **info)
{
	int retval;
	const struct FreeRTOS_params *param;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (const struct FreeRTOS_params *) rtos->rtos_specific_params;

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

static bool FreeRTOS_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols != NULL) &&
			(target->rtos->symbols[FreeRTOS_VAL_pxReadyTasksLists].address != 0)) {
		/* looks like FreeRTOS */
		return true;
	}
	return false;
}

static int FreeRTOS_create(struct target *target)
{
	int i = 0;
	while ((i < FREERTOS_NUM_PARAMS) &&
			(0 != strcmp(FreeRTOS_params_list[i].target_name, target->type->name))) {
		i++;
	}
	if (i >= FREERTOS_NUM_PARAMS) {
		LOG_ERROR("Could not find target in FreeRTOS compatibility list");
		return -1;
	}

	target->rtos->rtos_specific_params = (void *) &FreeRTOS_params_list[i];
	return 0;
}
