/***************************************************************************
 *   Copyright (C) 2014 by Marian Cingel                                   *
 *   cingel.marian@gmail.com                                               *
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

#include <stdint.h>
#include <helper/time_support.h>
#include <jtag/jtag.h>
#include "target/target.h"
#include "target/target_type.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "rtos_mqx_stackings.h"

/* constants */
#define MQX_THREAD_NAME_LENGTH			(255)
#define MQX_KERNEL_OFFSET_TDLIST		(0x0108)
#define MQX_KERNEL_OFFSET_SYSTEM_TASK	(0x0050)
#define MQX_KERNEL_OFFSET_ACTIVE_TASK	(0x001C)
#define MQX_KERNEL_OFFSET_CAPABILITY	(0x0000)
#define MQX_QUEUE_OFFSET_SIZE			(0x0008)
#define MQX_TASK_OFFSET_STATE			(0x0008)
#define MQX_TASK_OFFSET_ID				(0x000c)
#define MQX_TASK_OFFSET_TEMPLATE		(0x0068)
#define MQX_TASK_OFFSET_STACK			(0x0014)
#define MQX_TASK_OFFSET_TDLIST			(0x006C)
#define MQX_TASK_OFFSET_NEXT			(0x0000)
#define MQX_TASK_TEMPLATE_OFFSET_NAME	(0x0010)
#define MQX_TASK_OFFSET_ERROR_CODE		(0x005C)
#define MQX_TASK_STATE_MASK				(0xFFF)

/* types */
enum mqx_symbols {
	MQX_VAL_MQX_KERNEL_DATA,
	MQX_VAL_MQX_INIT_STRUCT,
};

enum mqx_arch {
	mqx_arch_cortexm,
};

struct mqx_params {
	const char *target_name;
	const enum mqx_arch target_arch;
	const struct rtos_register_stacking *stacking_info;
};

struct mqx_state {
	uint32_t state;
	char *name;
};

/* local data */
static const struct mqx_state mqx_states[] = {
	{ 0x0002, "READY" },
	{ 0x0003, "BLOCKED" },
	{ 0x0005, "RCV_SPECIFIC_BLOCKED" },
	{ 0x0007, "RCV_ANY_BLOCKED" },
	{ 0x0009, "DYING" },
	{ 0x000B, "UNHANDLED_INT_BLOCKED" },
	{ 0x000D, "SEND_BLOCKED" },
	{ 0x000F, "BREAKPOINT_BLOCKED" },
	{ 0x0211, "IO_BLOCKED" },
	{ 0x0021, "SEM_BLOCKED" },
	{ 0x0223, "MUTEX_BLOCKED" },
	{ 0x0025, "EVENT_BLOCKED" },
	{ 0x0229, "TASK_QUEUE_BLOCKED" },
	{ 0x042B, "LWSEM_BLOCKED" },
	{ 0x042D, "LWEVENT_BLOCKED" },
};

static const char * const mqx_symbol_list[] = {
	"_mqx_kernel_data",
	"MQX_init_struct",
	NULL
};

static const struct mqx_params mqx_params_list[] = {
	{ "cortex_m", mqx_arch_cortexm, &rtos_mqx_arm_v7m_stacking },
};

/*
 * Perform simple address check to avoid bus fault.
 */
static int mqx_valid_address_check(
	struct rtos *rtos,
	uint32_t address
)
{
	enum mqx_arch arch_type = ((struct mqx_params *)rtos->rtos_specific_params)->target_arch;
	const char *targetname = ((struct mqx_params *)rtos->rtos_specific_params)->target_name;

	/* Cortex-M address range */
	if (arch_type == mqx_arch_cortexm) {
		if (
			/* code and sram area */
			(address && address <= 0x3FFFFFFFu) ||
			/* external ram area*/
			(address >= 0x6000000u && address <= 0x9FFFFFFFu)
		) {
			return ERROR_OK;
		}
		return ERROR_FAIL;
	}
	LOG_ERROR("MQX RTOS - unknown architecture %s", targetname);
	return ERROR_FAIL;
}

/*
 * Wrapper of 'target_read_buffer' fn.
 * Include address check.
 */
static int mqx_target_read_buffer(
	struct target *target,
	uint32_t address,
	uint32_t size,
	uint8_t *buffer
)
{
	int status = mqx_valid_address_check(target->rtos, address);
	if (status != ERROR_OK) {
		LOG_WARNING("MQX RTOS - target address 0x%" PRIx32 " is not allowed to read", address);
		return status;
	}
	status = target_read_buffer(target, address, size, buffer);
	if (status != ERROR_OK) {
		LOG_ERROR("MQX RTOS - reading target address 0x%" PRIx32" failed", address);
		return status;
	}
	return ERROR_OK;
}

/*
 * Get symbol address if present
 */
static int mqx_get_symbol(
	struct rtos *rtos,
	enum mqx_symbols symbol,
	void *result
)
{
	/* TODO: additional check ?? */
	(*(int *)result) = (uint32_t)rtos->symbols[symbol].address;
	return ERROR_OK;
}

/*
 * Get value of struct member by passing
 * member offset, width and name (debug purpose)
 */
static int mqx_get_member(
	struct rtos *rtos,
	const uint32_t base_address,
	int32_t member_offset,
	int32_t member_width,
	const char *member_name,
	void *result
)
{
	int status = ERROR_FAIL;
	status = mqx_target_read_buffer(
		rtos->target, base_address + member_offset, member_width, result
	);
	if (status != ERROR_OK)
		LOG_WARNING("MQX RTOS - cannot read \"%s\" at address 0x%" PRIx32,
			    member_name, (uint32_t)(base_address + member_offset));
	return status;
}

/*
 * Check whether scheduler started
 */
static int mqx_is_scheduler_running(
	struct rtos *rtos
)
{
	uint32_t kernel_data_symbol = 0;
	uint32_t kernel_data_addr = 0;
	uint32_t system_td_addr = 0;
	uint32_t active_td_addr = 0;
	uint32_t capability_value = 0;

	/* get '_mqx_kernel_data' symbol */
	if (mqx_get_symbol(rtos, MQX_VAL_MQX_KERNEL_DATA, &kernel_data_symbol) != ERROR_OK)
		return ERROR_FAIL;

	/* get '_mqx_kernel_data' */
	if (mqx_get_member(rtos, kernel_data_symbol, 0, 4,
		"_mqx_kernel_data", &kernel_data_addr) != ERROR_OK)
		return ERROR_FAIL;

	/* return if '_mqx_kernel_data' is NULL or default 0xFFFFFFFF */
	if (kernel_data_addr == 0 || kernel_data_addr == (uint32_t)(-1))
		return ERROR_FAIL;
	/* get kernel_data->ADDRESSING_CAPABILITY */
	if (mqx_get_member(rtos, kernel_data_addr, MQX_KERNEL_OFFSET_CAPABILITY, 4,
		"kernel_data->ADDRESSING_CAPABILITY", (void *)&capability_value) != ERROR_OK)
		return ERROR_FAIL;

	/* check first member, the '_mqx_kernel_data->ADDRESSING_CAPABILITY'.
	   it suppose to be set to value 8 */
	if (capability_value != 8) {
		LOG_WARNING("MQX RTOS - value of '_mqx_kernel_data->ADDRESSING_CAPABILITY' contains invalid value");
		return ERROR_FAIL;
	}
	/* get active ptr */
	if (mqx_get_member(rtos, kernel_data_addr, MQX_KERNEL_OFFSET_ACTIVE_TASK, 4,
		"kernel_data->ACTIVE_PTR", (void *)&active_td_addr) != ERROR_OK)
		return ERROR_FAIL;

	/* active task is system task, scheduler has not not run yet */
	system_td_addr = kernel_data_addr + MQX_KERNEL_OFFSET_SYSTEM_TASK;
	if (active_td_addr == system_td_addr) {
		LOG_WARNING("MQX RTOS - scheduler does not run");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

/*
 * API function, return true if MQX is present
 */
static bool mqx_detect_rtos(
	struct target *target
)
{
	if (
		(target->rtos->symbols) &&
		(target->rtos->symbols[MQX_VAL_MQX_KERNEL_DATA].address != 0)
	) {
		return true;
	}
	return false;
}

/*
 * API function, pass MQX extra info to context data
 */
static int mqx_create(
	struct target *target
)
{
	/* check target name against supported architectures */
	for (unsigned int i = 0; i < ARRAY_SIZE(mqx_params_list); i++) {
		if (strcmp(mqx_params_list[i].target_name, target->type->name) == 0) {
			target->rtos->rtos_specific_params = (void *)&mqx_params_list[i];
			/* LOG_DEBUG("MQX RTOS - valid architecture: %s", target->type->name); */
			return 0;
		}
	}
	LOG_ERROR("MQX RTOS - could not find target \"%s\" in MQX compatibility list", target->type->name);
	return -1;
}

/*
 * API function, update list of threads
 */
static int mqx_update_threads(
	struct rtos *rtos
)
{
	uint32_t task_queue_addr = 0;
	uint32_t kernel_data_addr = 0;
	uint16_t task_queue_size = 0;
	uint32_t active_td_addr = 0;

	if (!rtos->rtos_specific_params)
		return -3;

	if (!rtos->symbols)
		return -4;

	/* clear old data */
	rtos_free_threadlist(rtos);
	/* check scheduler */
	if (mqx_is_scheduler_running(rtos) != ERROR_OK)
		return ERROR_FAIL;
	/* get kernel_data symbol */
	if (mqx_get_symbol(rtos, MQX_VAL_MQX_KERNEL_DATA, &kernel_data_addr) != ERROR_OK)
		return ERROR_FAIL;

	/* read kernel_data */
	if (mqx_get_member(rtos, kernel_data_addr, 0, 4,
		"_mqx_kernel_data", &kernel_data_addr) != ERROR_OK)
		return ERROR_FAIL;

	/* get task queue address */
	task_queue_addr = kernel_data_addr + MQX_KERNEL_OFFSET_TDLIST;
	/* get task queue size */
	if (mqx_get_member(rtos, task_queue_addr, MQX_QUEUE_OFFSET_SIZE, 2,
		"kernel_data->TD_LIST.SIZE", &task_queue_size) != ERROR_OK)
		return ERROR_FAIL;

	/* get active ptr */
	if (mqx_get_member(rtos, kernel_data_addr, MQX_KERNEL_OFFSET_ACTIVE_TASK, 4,
		"kernel_data->ACTIVE_PTR", (void *)&active_td_addr) != ERROR_OK)
		return ERROR_FAIL;

	/* setup threads info */
	rtos->thread_count = task_queue_size;
	rtos->current_thread = 0;
	rtos->thread_details = calloc(rtos->thread_count, sizeof(struct thread_detail));
	if (!rtos->thread_details)
		return ERROR_FAIL;

	/*	loop over each task and setup thread details,
		the current_taskpool_addr is set to queue head
		NOTE: debugging functions task create/destroy
		might cause to show invalid data.
	*/
	for (
		uint32_t i = 0, taskpool_addr = task_queue_addr;
		i < (uint32_t)rtos->thread_count;
		i++
	) {
		uint8_t task_name[MQX_THREAD_NAME_LENGTH + 1];
		uint32_t task_addr = 0, task_template = 0, task_state = 0;
		uint32_t task_name_addr = 0, task_id = 0, task_errno = 0;
		uint32_t state_index = 0;
		uint32_t extra_info_length = 0;
		char *state_name = "Unknown";

		/* set current taskpool address */
		if (mqx_get_member(rtos, taskpool_addr, MQX_TASK_OFFSET_NEXT, 4,
			"td_struct_ptr->NEXT", &taskpool_addr) != ERROR_OK)
			return ERROR_FAIL;

		/* get task address from taskpool */
		task_addr = taskpool_addr - MQX_TASK_OFFSET_TDLIST;
		/* get address of 'td_struct_ptr->TEMPLATE_LIST_PTR' */
		if (mqx_get_member(rtos, task_addr, MQX_TASK_OFFSET_TEMPLATE, 4,
			"td_struct_ptr->TEMPLATE_LIST_PTR", &task_template) != ERROR_OK)
			return ERROR_FAIL;

		/* get address of 'td_struct_ptr->TEMPLATE_LIST_PTR->NAME' */
		if (mqx_get_member(rtos, task_template, MQX_TASK_TEMPLATE_OFFSET_NAME, 4,
			"td_struct_ptr->TEMPLATE_LIST_PTR->NAME", &task_name_addr) != ERROR_OK)
			return ERROR_FAIL;

		/* get value of 'td_struct->TEMPLATE_LIST_PTR->NAME' */
		if (mqx_get_member(rtos, task_name_addr, 0, MQX_THREAD_NAME_LENGTH,
			"*td_struct_ptr->TEMPLATE_LIST_PTR->NAME", task_name) != ERROR_OK)
			return ERROR_FAIL;

		/* always terminate last character by force,
		   otherwise openocd might fail if task_name
		   has corrupted data */
		task_name[MQX_THREAD_NAME_LENGTH] = '\0';
		/* get value of 'td_struct_ptr->TASK_ID' */
		if (mqx_get_member(rtos, task_addr, MQX_TASK_OFFSET_ID, 4,
			"td_struct_ptr->TASK_ID", &task_id) != ERROR_OK)
			return ERROR_FAIL;

		/* get task errno */
		if (mqx_get_member(rtos, task_addr, MQX_TASK_OFFSET_ERROR_CODE, 4,
			"td_struct_ptr->TASK_ERROR_CODE", &task_errno) != ERROR_OK)
			return ERROR_FAIL;

		/* get value of 'td_struct_ptr->STATE' */
		if (mqx_get_member(rtos, task_addr, MQX_TASK_OFFSET_STATE, 4,
			"td_struct_ptr->STATE", &task_state) != ERROR_OK)
			return ERROR_FAIL;

		task_state &= MQX_TASK_STATE_MASK;
		/* and search for defined state */
		for (state_index = 0; state_index < ARRAY_SIZE(mqx_states); state_index++) {
			if (mqx_states[state_index].state == task_state) {
				state_name = mqx_states[state_index].name;
				break;
			}
		}

		/* setup thread details struct */
		rtos->thread_details[i].threadid = task_id;
		rtos->thread_details[i].exists = true;
		/* set thread name */
		rtos->thread_details[i].thread_name_str = malloc(strlen((void *)task_name) + 1);
		if (!rtos->thread_details[i].thread_name_str)
			return ERROR_FAIL;
		strcpy(rtos->thread_details[i].thread_name_str, (void *)task_name);
		/* set thread extra info
		 * - task state
		 * - task address
		 * - task errno
		 * calculate length as:
		 * state length + address length + errno length + formatter length
		 */
		extra_info_length += strlen((void *)state_name) + 7 + 13 + 8 + 15 + 8;
		rtos->thread_details[i].extra_info_str = malloc(extra_info_length + 1);
		if (!rtos->thread_details[i].extra_info_str)
			return ERROR_FAIL;
		snprintf(rtos->thread_details[i].extra_info_str, extra_info_length,
			 "State: %s, Address: 0x%" PRIx32 ",  Error Code: %" PRIu32,
			 state_name, task_addr, task_errno
		);
		/* set active thread */
		if (active_td_addr == task_addr)
			rtos->current_thread = task_id;
	}
	return ERROR_OK;
}

/*
 * API function, get info of selected thread
 */
static int mqx_get_thread_reg_list(
	struct rtos *rtos,
	int64_t thread_id,
	struct rtos_reg **reg_list,
	int *num_regs
)
{
	int64_t stack_ptr = 0;
	uint32_t my_task_addr = 0;
	uint32_t task_queue_addr = 0;
	uint32_t task_queue_size = 0;
	uint32_t kernel_data_addr = 0;

	if (thread_id == 0) {
		LOG_ERROR("MQX RTOS - invalid threadid: 0x%X", (int)thread_id);
		return ERROR_FAIL;
	}
	if (mqx_is_scheduler_running(rtos) != ERROR_OK)
		return ERROR_FAIL;
	/* get kernel_data symbol */
	if (mqx_get_symbol(rtos, MQX_VAL_MQX_KERNEL_DATA, &kernel_data_addr) != ERROR_OK)
		return ERROR_FAIL;

	/* read kernel_data */
	if (mqx_get_member(rtos, kernel_data_addr, 0, 4,
		"_mqx_kernel_data", &kernel_data_addr) != ERROR_OK)
		return ERROR_FAIL;

	/* get task queue address */
	task_queue_addr = kernel_data_addr + MQX_KERNEL_OFFSET_TDLIST;
	/* get task queue size */
	if (mqx_get_member(rtos, task_queue_addr, MQX_QUEUE_OFFSET_SIZE, 2,
		"kernel_data->TD_LIST.SIZE", &task_queue_size) != ERROR_OK)
		return ERROR_FAIL;

	/* search for taskid */
	for (
		uint32_t i = 0, taskpool_addr = task_queue_addr;
		i < (uint32_t)rtos->thread_count;
		i++
	) {
		uint32_t tmp_address = 0, task_addr = 0;
		uint32_t task_id = 0;
		/* set current taskpool address */
		tmp_address = taskpool_addr;
		if (mqx_get_member(rtos, tmp_address, MQX_TASK_OFFSET_NEXT, 4,
			"td_struct_ptr->NEXT", &taskpool_addr) != ERROR_OK)
			return ERROR_FAIL;

		/* get task address from taskpool */
		task_addr = taskpool_addr - MQX_TASK_OFFSET_TDLIST;
		/* get value of td_struct->TASK_ID */
		if (mqx_get_member(rtos, task_addr, MQX_TASK_OFFSET_ID, 4,
			"td_struct_ptr->TASK_ID", &task_id) != ERROR_OK)
			return ERROR_FAIL;

		/* found taskid, break */
		if (task_id == thread_id) {
			my_task_addr = task_addr;
			break;
		}
	}
	if (!my_task_addr) {
		LOG_ERROR("MQX_RTOS - threadid %" PRId64 " does not match any task", thread_id);
		return ERROR_FAIL;
	}
	/* get task stack head address */
	if (mqx_get_member(rtos, my_task_addr, MQX_TASK_OFFSET_STACK, 4,
		"task->STACK_PTR", &stack_ptr) != ERROR_OK)
		return ERROR_FAIL;

	return rtos_generic_stack_read(
		rtos->target, ((struct mqx_params *)rtos->rtos_specific_params)->stacking_info, stack_ptr, reg_list, num_regs
	);
}

/* API function, export list of required symbols */
static int mqx_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	*symbol_list = calloc(ARRAY_SIZE(mqx_symbol_list), sizeof(struct symbol_table_elem));
	if (!*symbol_list)
		return ERROR_FAIL;
	/* export required symbols */
	for (int i = 0; i < (int)(ARRAY_SIZE(mqx_symbol_list)); i++)
		(*symbol_list)[i].symbol_name = mqx_symbol_list[i];
	return ERROR_OK;
}

struct rtos_type mqx_rtos = {
	.name = "mqx",
	.detect_rtos = mqx_detect_rtos,
	.create = mqx_create,
	.update_threads = mqx_update_threads,
	.get_thread_reg_list = mqx_get_thread_reg_list,
	.get_symbol_list_to_lookup = mqx_get_symbol_list_to_lookup,
};
