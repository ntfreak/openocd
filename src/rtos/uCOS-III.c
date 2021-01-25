/***************************************************************************
 *   Copyright (C) 2017 by Square, Inc.                                    *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/time_support.h>
#include <helper/types.h>
#include <rtos/rtos.h>
#include <target/target.h>
#include <target/target_type.h>

#include "rtos_ucos_iii_stackings.h"

#ifndef UCOS_III_MAX_STRLEN
#define UCOS_III_MAX_STRLEN 64
#endif

#ifndef UCOS_III_MAX_THREADS
#define UCOS_III_MAX_THREADS 256
#endif

struct uCOS_III_params {
	const char *target_name;
	const unsigned char pointer_width;
	symbol_address_t thread_stack_offset;
	symbol_address_t thread_name_offset;
	symbol_address_t thread_state_offset;
	symbol_address_t thread_priority_offset;
	symbol_address_t thread_prev_offset;
	symbol_address_t thread_next_offset;
	bool thread_offsets_updated;
	size_t threadid_start;
	const struct rtos_register_stacking *stacking_info;
	size_t num_threads;
	symbol_address_t threads[];
};

static const struct uCOS_III_params uCOS_III_params_list[] = {
	{
		"cortex_m",							/* target_name */
		sizeof(uint32_t),					/* pointer_width */
		0,									/* thread_stack_offset */
		0,									/* thread_name_offset */
		0,									/* thread_state_offset */
		0,									/* thread_priority_offset */
		0,									/* thread_prev_offset */
		0,									/* thread_next_offset */
		false,								/* thread_offsets_updated */
		1,									/* threadid_start */
		&rtos_uCOS_III_Cortex_M_stacking,	/* stacking_info */
		0,									/* num_threads */
	},
	{
		"esirisc",							/* target_name */
		sizeof(uint32_t),					/* pointer_width */
		0,									/* thread_stack_offset */
		0,									/* thread_name_offset */
		0,									/* thread_state_offset */
		0,									/* thread_priority_offset */
		0,									/* thread_prev_offset */
		0,									/* thread_next_offset */
		false,								/* thread_offsets_updated */
		1,									/* threadid_start */
		&rtos_uCOS_III_eSi_RISC_stacking,	/* stacking_info */
		0,									/* num_threads */
	},
};

static const char * const uCOS_III_symbol_list[] = {
	"OSRunning",
	"OSTCBCurPtr",
	"OSTaskDbgListPtr",
	"OSTaskQty",

	/* also see: contrib/rtos-helpers/uCOS-III-openocd.c */
	"openocd_OS_TCB_StkPtr_offset",
	"openocd_OS_TCB_NamePtr_offset",
	"openocd_OS_TCB_TaskState_offset",
	"openocd_OS_TCB_Prio_offset",
	"openocd_OS_TCB_DbgPrevPtr_offset",
	"openocd_OS_TCB_DbgNextPtr_offset",
	NULL
};

enum uCOS_III_symbol_values {
	uCOS_III_VAL_OSRunning,
	uCOS_III_VAL_OSTCBCurPtr,
	uCOS_III_VAL_OSTaskDbgListPtr,
	uCOS_III_VAL_OSTaskQty,

	/* also see: contrib/rtos-helpers/uCOS-III-openocd.c */
	uCOS_III_VAL_OS_TCB_StkPtr_offset,
	uCOS_III_VAL_OS_TCB_NamePtr_offset,
	uCOS_III_VAL_OS_TCB_TaskState_offset,
	uCOS_III_VAL_OS_TCB_Prio_offset,
	uCOS_III_VAL_OS_TCB_DbgPrevPtr_offset,
	uCOS_III_VAL_OS_TCB_DbgNextPtr_offset,
};

static const char * const uCOS_III_thread_state_list[] = {
	"Ready",
	"Delay",
	"Pend",
	"Pend Timeout",
	"Suspended",
	"Delay Suspended",
	"Pend Suspended",
	"Pend Timeout Suspended",
};

static int uCOS_III_find_or_create_thread(struct rtos *rtos, symbol_address_t thread_address,
		threadid_t *threadid)
{
	struct uCOS_III_params *params = rtos->rtos_specific_params;
	size_t thread_index;

	for (thread_index = 0; thread_index < params->num_threads; thread_index++)
		if (params->threads[thread_index] == thread_address)
			goto found;

	if (params->num_threads == UCOS_III_MAX_THREADS) {
		LOG_WARNING("uCOS-III: too many threads; increase UCOS_III_MAX_THREADS");
		return ERROR_FAIL;
	}

	params->threads[thread_index] = thread_address;
	params->num_threads++;
found:
	*threadid = thread_index + params->threadid_start;
	return ERROR_OK;
}

static int uCOS_III_find_thread_address(struct rtos *rtos, threadid_t threadid,
		symbol_address_t *thread_address)
{
	struct uCOS_III_params *params = rtos->rtos_specific_params;
	size_t thread_index;

	thread_index = threadid - params->threadid_start;
	if (thread_index >= params->num_threads) {
		LOG_ERROR("uCOS-III: failed to find thread address");
		return ERROR_FAIL;
	}

	*thread_address = params->threads[thread_index];
	return ERROR_OK;
}

static int uCOS_III_find_last_thread_address(struct rtos *rtos, symbol_address_t *thread_address)
{
	struct uCOS_III_params *params = rtos->rtos_specific_params;
	int retval;

	/* read the thread list head */
	symbol_address_t thread_list_address = 0;

	retval = target_read_memory(rtos->target,
			rtos->symbols[uCOS_III_VAL_OSTaskDbgListPtr].address,
			params->pointer_width,
			1,
			(void *)&thread_list_address);
	if (retval != ERROR_OK) {
		LOG_ERROR("uCOS-III: failed to read thread list address");
		return retval;
	}

	/* advance to end of thread list */
	do {
		*thread_address = thread_list_address;

		retval = target_read_memory(rtos->target,
				thread_list_address + params->thread_next_offset,
				params->pointer_width,
				1,
				(void *)&thread_list_address);
		if (retval != ERROR_OK) {
			LOG_ERROR("uCOS-III: failed to read next thread address");
			return retval;
		}
	} while (thread_list_address != 0);

	return ERROR_OK;
}

static int uCOS_III_update_thread_offsets(struct rtos *rtos)
{
	struct uCOS_III_params *params = rtos->rtos_specific_params;

	if (params->thread_offsets_updated)
		return ERROR_OK;

	const struct thread_offset_map {
		enum uCOS_III_symbol_values symbol_value;
		symbol_address_t *thread_offset;
	} thread_offset_maps[] = {
		{
			uCOS_III_VAL_OS_TCB_StkPtr_offset,
			&params->thread_stack_offset,
		},
		{
			uCOS_III_VAL_OS_TCB_NamePtr_offset,
			&params->thread_name_offset,
		},
		{
			uCOS_III_VAL_OS_TCB_TaskState_offset,
			&params->thread_state_offset,
		},
		{
			uCOS_III_VAL_OS_TCB_Prio_offset,
			&params->thread_priority_offset,
		},
		{
			uCOS_III_VAL_OS_TCB_DbgPrevPtr_offset,
			&params->thread_prev_offset,
		},
		{
			uCOS_III_VAL_OS_TCB_DbgNextPtr_offset,
			&params->thread_next_offset,
		},
	};

	for (size_t i = 0; i < ARRAY_SIZE(thread_offset_maps); i++) {
		const struct thread_offset_map *thread_offset_map = &thread_offset_maps[i];

		int retval = target_read_memory(rtos->target,
				rtos->symbols[thread_offset_map->symbol_value].address,
				params->pointer_width,
				1,
				(void *)thread_offset_map->thread_offset);
		if (retval != ERROR_OK) {
			LOG_ERROR("uCOS-III: failed to read thread offset");
			return retval;
		}
	}

	params->thread_offsets_updated = true;
	return ERROR_OK;
}

static bool uCOS_III_detect_rtos(struct target *target)
{
	return target->rtos->symbols != NULL &&
			target->rtos->symbols[uCOS_III_VAL_OSRunning].address != 0;
}

static int uCOS_III_reset_handler(struct target *target, enum target_reset_mode reset_mode, void *priv)
{
	struct uCOS_III_params *params = target->rtos->rtos_specific_params;

	params->thread_offsets_updated = false;
	params->num_threads = 0;

	return ERROR_OK;
}

static int uCOS_III_create(struct target *target)
{
	struct uCOS_III_params *params;

	for (size_t i = 0; i < ARRAY_SIZE(uCOS_III_params_list); i++)
		if (strcmp(uCOS_III_params_list[i].target_name, target->type->name) == 0) {
			params = malloc(sizeof(*params) + (UCOS_III_MAX_THREADS * sizeof(*params->threads)));
			if (params == NULL) {
				LOG_ERROR("uCOS-III: out of memory");
				return ERROR_FAIL;
			}

			memcpy(params, &uCOS_III_params_list[i], sizeof(uCOS_III_params_list[i]));
			target->rtos->rtos_specific_params = (void *)params;

			target_register_reset_callback(uCOS_III_reset_handler, NULL);

			return ERROR_OK;
		}

	LOG_ERROR("uCOS-III: target not supported: %s", target->type->name);
	return ERROR_FAIL;
}

static int uCOS_III_update_threads(struct rtos *rtos)
{
	struct uCOS_III_params *params = rtos->rtos_specific_params;
	int retval;

	if (rtos->symbols == NULL) {
		LOG_ERROR("uCOS-III: symbol list not loaded");
		return ERROR_FAIL;
	}

	/* free previous thread details */
	rtos_free_threadlist(rtos);

	/* verify RTOS is running */
	uint8_t rtos_running;

	retval = target_read_u8(rtos->target,
			rtos->symbols[uCOS_III_VAL_OSRunning].address,
			&rtos_running);
	if (retval != ERROR_OK) {
		LOG_ERROR("uCOS-III: failed to read RTOS running");
		return retval;
	}

	if (rtos_running != 1 && rtos_running != 0) {
		LOG_ERROR("uCOS-III: invalid RTOS running value");
		return ERROR_FAIL;
	}

	if (!rtos_running) {
		rtos->thread_details = calloc(1, sizeof(struct thread_detail));
		if (rtos->thread_details == NULL) {
			LOG_ERROR("uCOS-III: out of memory");
			return ERROR_FAIL;
		}

		rtos->thread_count = 1;
		rtos->thread_details->threadid = 0;
		rtos->thread_details->exists = true;
		rtos->current_thread = 0;

		return ERROR_OK;
	}

	/* update thread offsets */
	retval = uCOS_III_update_thread_offsets(rtos);
	if (retval != ERROR_OK) {
		LOG_ERROR("uCOS-III: failed to update thread offsets");
		return retval;
	}

	/* read current thread address */
	symbol_address_t current_thread_address = 0;

	retval = target_read_memory(rtos->target,
			rtos->symbols[uCOS_III_VAL_OSTCBCurPtr].address,
			params->pointer_width,
			1,
			(void *)&current_thread_address);
	if (retval != ERROR_OK) {
		LOG_ERROR("uCOS-III: failed to read current thread address");
		return retval;
	}

	/* read number of tasks */
	retval = target_read_u16(rtos->target,
			rtos->symbols[uCOS_III_VAL_OSTaskQty].address,
			(void *)&rtos->thread_count);
	if (retval != ERROR_OK) {
		LOG_ERROR("uCOS-III: failed to read thread count");
		return retval;
	}

	rtos->thread_details = calloc(rtos->thread_count, sizeof(struct thread_detail));
	if (rtos->thread_details == NULL) {
		LOG_ERROR("uCOS-III: out of memory");
		return ERROR_FAIL;
	}

	/*
	 * uC/OS-III adds tasks in LIFO order; advance to the end of the
	 * list and work backwards to preserve the intended order.
	 */
	symbol_address_t thread_address = 0;

	retval = uCOS_III_find_last_thread_address(rtos, &thread_address);
	if (retval != ERROR_OK) {
		LOG_ERROR("uCOS-III: failed to find last thread address");
		return retval;
	}

	for (int i = 0; i < rtos->thread_count; i++) {
		struct thread_detail *thread_detail = &rtos->thread_details[i];
		char thread_str_buffer[UCOS_III_MAX_STRLEN + 1];

		/* find or create new threadid */
		retval = uCOS_III_find_or_create_thread(rtos, thread_address, &thread_detail->threadid);
		if (retval != ERROR_OK) {
			LOG_ERROR("uCOS-III: failed to find or create thread");
			return retval;
		}

		if (thread_address == current_thread_address)
			rtos->current_thread = thread_detail->threadid;

		thread_detail->exists = true;

		/* read thread name */
		symbol_address_t thread_name_address = 0;

		retval = target_read_memory(rtos->target,
				thread_address + params->thread_name_offset,
				params->pointer_width,
				1,
				(void *)&thread_name_address);
		if (retval != ERROR_OK) {
			LOG_ERROR("uCOS-III: failed to name address");
			return retval;
		}

		retval = target_read_buffer(rtos->target,
				thread_name_address,
				sizeof(thread_str_buffer),
				(void *)thread_str_buffer);
		if (retval != ERROR_OK) {
			LOG_ERROR("uCOS-III: failed to read thread name");
			return retval;
		}

		thread_str_buffer[sizeof(thread_str_buffer) - 1] = '\0';
		thread_detail->thread_name_str = strdup(thread_str_buffer);

		/* read thread extra info */
		uint8_t thread_state;
		uint8_t thread_priority;

		retval = target_read_u8(rtos->target,
				thread_address + params->thread_state_offset,
				&thread_state);
		if (retval != ERROR_OK) {
			LOG_ERROR("uCOS-III: failed to read thread state");
			return retval;
		}

		retval = target_read_u8(rtos->target,
				thread_address + params->thread_priority_offset,
				&thread_priority);
		if (retval != ERROR_OK) {
			LOG_ERROR("uCOS-III: failed to read thread priority");
			return retval;
		}

		const char *thread_state_str;

		if (thread_state < ARRAY_SIZE(uCOS_III_thread_state_list))
			thread_state_str = uCOS_III_thread_state_list[thread_state];
		else
			thread_state_str = "Unknown";

		snprintf(thread_str_buffer, sizeof(thread_str_buffer), "State: %s, Priority: %d",
				thread_state_str, thread_priority);
		thread_detail->extra_info_str = strdup(thread_str_buffer);

		/* read previous thread address */
		retval = target_read_memory(rtos->target,
				thread_address + params->thread_prev_offset,
				params->pointer_width,
				1,
				(void *)&thread_address);
		if (retval != ERROR_OK) {
			LOG_ERROR("uCOS-III: failed to read previous thread address");
			return retval;
		}
	}

	return ERROR_OK;
}

static int uCOS_III_get_thread_reg_list(struct rtos *rtos, threadid_t threadid,
		struct rtos_reg **reg_list, int *num_regs)
{
	struct uCOS_III_params *params = rtos->rtos_specific_params;
	int retval;

	/* find thread address for threadid */
	symbol_address_t thread_address = 0;

	retval = uCOS_III_find_thread_address(rtos, threadid, &thread_address);
	if (retval != ERROR_OK) {
		LOG_ERROR("uCOS-III: failed to find thread address");
		return retval;
	}

	/* read thread stack address */
	symbol_address_t stack_address = 0;

	retval = target_read_memory(rtos->target,
			thread_address + params->thread_stack_offset,
			params->pointer_width,
			1,
			(void *)&stack_address);
	if (retval != ERROR_OK) {
		LOG_ERROR("uCOS-III: failed to read stack address");
		return retval;
	}

	return rtos_generic_stack_read(rtos->target,
			params->stacking_info,
			stack_address,
			reg_list,
			num_regs);
}

static int uCOS_III_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	*symbol_list = calloc(ARRAY_SIZE(uCOS_III_symbol_list), sizeof(struct symbol_table_elem));
	if (*symbol_list == NULL) {
		LOG_ERROR("uCOS-III: out of memory");
		return ERROR_FAIL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(uCOS_III_symbol_list); i++)
		(*symbol_list)[i].symbol_name = uCOS_III_symbol_list[i];

	return ERROR_OK;
}

const struct rtos_type uCOS_III_rtos = {
	.name = "uCOS-III",
	.detect_rtos = uCOS_III_detect_rtos,
	.create = uCOS_III_create,
	.update_threads = uCOS_III_update_threads,
	.get_thread_reg_list = uCOS_III_get_thread_reg_list,
	.get_symbol_list_to_lookup = uCOS_III_get_symbol_list_to_lookup,
};
