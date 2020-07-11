/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2018 National Instruments Corp
 * Author: Moritz Fischer <moritz.fischer@ettus.com>
 *
 * Chromium-EC RTOS Task Awareness
 */

#include <rtos/rtos.h>
#include <target/target.h>
#include <target/target_type.h>

#include "rtos_standard_stackings.h"

#define CROS_EC_MAX_TASKS 32
#define CROS_EC_MAX_NAME 200
#define CROS_EC_IDLE_STRING "<< idle >>"
#define BIT(x) (1 << (x))

struct chromium_ec_params {
	const char *target_name;
	size_t ptr_size;
	off_t task_offset_next;
	off_t task_offset_sp;
	off_t task_offset_events;
	off_t task_offset_runtime;
	const struct rtos_register_stacking *stacking;
};

static const struct chromium_ec_params chromium_ec_params_list[] = {
	{
		.target_name = "hla_target",
		.ptr_size = 4,
		.task_offset_next = 24,
		.task_offset_sp = 0,
		.task_offset_events = 4,
		.task_offset_runtime = 8,
		.stacking = &rtos_standard_Cortex_M3_stacking,

	},
	{
		.target_name = "cortex_m",
		.ptr_size = 4,
		.task_offset_next = 24,
		.task_offset_sp = 0,
		.task_offset_events = 4,
		.task_offset_runtime = 8,
		.stacking = &rtos_standard_Cortex_M3_stacking,
	},
};

static const char * const chromium_ec_symbol_list[] = {
	"start_called",
	"current_task",
	"tasks",
	"tasks_enabled",
	"tasks_ready",
	"task_names",
	"build_info",
	NULL,
};

enum chromium_ec_symbol_values {
	CHROMIUM_EC_VAL_start_called = 0,
	CHROMIUM_EC_VAL_current_task,
	CHROMIUM_EC_VAL_tasks,
	CHROMIUM_EC_VAL_tasks_enabled,
	CHROMIUM_EC_VAL_tasks_ready,
	CHROMIUM_EC_VAL_task_names,
	CHROMIUM_EC_VAL_build_info,

	CHROMIUM_EC_VAL_COUNT,
};

#define CROS_EC_MAX_BUILDINFO 512

static bool chromium_ec_detect_rtos(struct target *target)
{
	char build_info_buf[CROS_EC_MAX_BUILDINFO];
	enum chromium_ec_symbol_values sym;
	int ret;

	if (!target || !target->rtos || !target->rtos->symbols)
		return false;

	for (sym = CHROMIUM_EC_VAL_start_called;
	     sym < CHROMIUM_EC_VAL_COUNT; sym++) {
		if (target->rtos->symbols[sym].address) {
			LOG_DEBUG("Chromium-EC: Symbol \"%s\" found",
				 chromium_ec_symbol_list[sym]);
		} else {
			LOG_ERROR("Chromium-EC: Symbol \"%s\" missing",
				 chromium_ec_symbol_list[sym]);
			return false;
		}
	}

	ret = target_read_buffer(target,
				 target->rtos->symbols[CHROMIUM_EC_VAL_build_info].address,
				 sizeof(build_info_buf),
				 (uint8_t *)build_info_buf);

	if (ret != ERROR_OK)
		return false;

	LOG_INFO("Chromium-EC: Buildinfo: %s", build_info_buf);

	return target->rtos->symbols &&
	       target->rtos->symbols[CHROMIUM_EC_VAL_start_called].address;
}

static int chromium_ec_create(struct target *target)
{
	struct chromium_ec_params *params;
	size_t t;

	for (t = 0; t < ARRAY_SIZE(chromium_ec_params_list); t++)
		if (!strcmp(chromium_ec_params_list[t].target_name, target->type->name)) {
			params = malloc(sizeof(*params));
			if (!params) {
				LOG_ERROR("Chromium-EC: out of memory");
				return ERROR_FAIL;
			}

			memcpy(params, &chromium_ec_params_list[t], sizeof(*params));
			target->rtos->rtos_specific_params = (void *)params;
			target->rtos->current_thread = 0;
			target->rtos->thread_details = NULL;
			target->rtos->thread_count = 0;

			LOG_INFO("Chromium-EC: Using target: %s", target->type->name);
			return ERROR_OK;
		}

	LOG_ERROR("Chromium-EC: target not supported: %s", target->type->name);
	return ERROR_FAIL;
}

static int chromium_ec_get_current_task_ptr(struct rtos *rtos, uint32_t *current_task)
{
	if (!rtos || !rtos->symbols)
		return ERROR_FAIL;

	return target_read_u32(rtos->target,
			       rtos->symbols[CHROMIUM_EC_VAL_current_task].address,
			       current_task);
}

static int chromium_ec_get_num_tasks(struct rtos *rtos, int *num_tasks)
{
	uint32_t tasks_enabled;
	int ret, t, found;

	ret = target_read_u32(rtos->target,
			      rtos->symbols[CHROMIUM_EC_VAL_tasks_enabled].address,
			      &tasks_enabled);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to determine #of tasks");
		return ret;
	}

	found = 0;
	for (t = 0; t < CROS_EC_MAX_TASKS; t++)
		if (tasks_enabled & BIT(t))
			found++;

	*num_tasks = found;

	return ERROR_OK;
}

static int chromium_ec_update_threads(struct rtos *rtos)
{
	uint32_t tasks_enabled, tasks_ready, start_called;
	uint32_t current_task, thread_ptr, name_ptr;
	char thread_str_buf[CROS_EC_MAX_NAME];
	int ret, t, num_tasks, tasks_found;
	struct chromium_ec_params *params;
	uint8_t runtime_buf[8];
	uint64_t runtime;
	uint32_t events;

	params = rtos->rtos_specific_params;
	if (!params)
		return ERROR_FAIL;

	if (!rtos->symbols)
		return ERROR_FAIL;

	num_tasks = 0;
	ret = chromium_ec_get_num_tasks(rtos, &num_tasks);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to get number of tasks");
		return ret;
	}

	current_task = 0;
	ret = chromium_ec_get_current_task_ptr(rtos, &current_task);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to get current task");
		return ret;
	}
	LOG_DEBUG("Current task: %lx tasks_found: %d",
		  (unsigned long)current_task,
		  num_tasks);

	/* set current task to what we read */
	rtos->current_thread = current_task;

	/* Nuke the old tasks */
	rtos_free_threadlist(rtos);

	/* One check if task switching has started ... */
	start_called = 0;
	ret = target_read_u32(rtos->target, rtos->symbols[CHROMIUM_EC_VAL_start_called].address,
			      &start_called);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to load start_called");
		return ret;
	}

	if (!rtos->current_thread || !num_tasks || !start_called) {
		num_tasks++;

		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * num_tasks);
		rtos->thread_details->threadid = 1;
		rtos->thread_details->exists = true;
		rtos->thread_details->extra_info_str = NULL;
		rtos->thread_details->thread_name_str = strdup("Current Execution");

		if (!num_tasks || !start_called) {
			rtos->thread_count = 1;
			return ERROR_OK;
		}
	} else {
		/* create space for new thread details */
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * num_tasks);
	}

	tasks_enabled = 0;
	ret = target_read_u32(rtos->target, rtos->symbols[CHROMIUM_EC_VAL_tasks_enabled].address,
			      &tasks_enabled);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to load tasks_enabled");
		return ret;
	}

	tasks_ready = 0;
	ret = target_read_u32(rtos->target, rtos->symbols[CHROMIUM_EC_VAL_tasks_ready].address,
			      &tasks_ready);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to load tasks_ready");
		return ret;
	}

	thread_ptr = rtos->symbols[CHROMIUM_EC_VAL_tasks].address;

	tasks_found = 0;
	for (t = 0; t < CROS_EC_MAX_TASKS; t++) {
		if (!(tasks_enabled & BIT(t)))
			continue;

		if (thread_ptr == current_task)
			rtos->current_thread = thread_ptr;

		rtos->thread_details[tasks_found].threadid = thread_ptr;
		ret = target_read_u32(rtos->target,
					 rtos->symbols[CHROMIUM_EC_VAL_task_names].address +
					 params->ptr_size * t, &name_ptr);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to read name_ptr");
			return ret;
		}

		/* read name buffer */
		ret = target_read_buffer(rtos->target, name_ptr, CROS_EC_MAX_NAME,
					(uint8_t *)thread_str_buf);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to read task name");
			return ret;
		}

		/* sanitize string, gdb chokes on "<< idle >>" */
		if (thread_str_buf[CROS_EC_MAX_NAME - 1] != '\0')
			thread_str_buf[CROS_EC_MAX_NAME - 1] = '\0';
		if (!strncmp(thread_str_buf, CROS_EC_IDLE_STRING, CROS_EC_MAX_NAME))
		    rtos->thread_details[tasks_found].thread_name_str = strdup("IDLE");
		else
		    rtos->thread_details[tasks_found].thread_name_str = strdup(thread_str_buf);

		events = 0;
		ret = target_read_u32(rtos->target,
				      thread_ptr + params->task_offset_events,
				      &events);
		if (ret != ERROR_OK)
			LOG_ERROR("Failed to get task %d's events", t);

		/* this is a bit kludgy but will do for now */
		ret = target_read_buffer(rtos->target,
					 thread_ptr + params->task_offset_runtime,
					 sizeof(runtime_buf), runtime_buf);
		if (ret != ERROR_OK)
			LOG_ERROR("Failed to get task %d's runtime", t);
		runtime =  target_buffer_get_u64(rtos->target, runtime_buf);

		/* Priority is simply the position in the array */
		if (thread_ptr == current_task)
			snprintf(thread_str_buf, sizeof(thread_str_buf),
				 "State: Running, Priority: %u, Events: %" PRIx32 ", Runtime: %" PRIu64 "\n",
				 t, events, runtime);
		else
			snprintf(thread_str_buf, sizeof(thread_str_buf),
				 "State: %s, Priority: %u, Events: %" PRIx32 ", Runtime: %" PRIu64 "\n",
				 tasks_ready & BIT(t) ? "Ready" : "Waiting", t,
				 events, runtime);

		rtos->thread_details[tasks_found].extra_info_str = strdup(thread_str_buf);
		rtos->thread_details[tasks_found].exists = true;

		thread_ptr += params->task_offset_next;

		tasks_found++;
	}

	rtos->thread_count = tasks_found;

	return ERROR_OK;
}

static int chromium_ec_get_thread_reg_list(struct rtos *rtos,
					   threadid_t threadid,
					   struct rtos_reg **reg_list,
					   int *num_regs)
{
	struct chromium_ec_params *params = rtos->rtos_specific_params;
	uint32_t stack_ptr = 0;
	int ret, t;

	for (t = 0; t < rtos->thread_count; t++)
		if (threadid == rtos->thread_details[t].threadid)
			break;

	/* if we didn't find threadid, bail */
	if (t == rtos->thread_count)
		return ERROR_FAIL;

	ret = target_read_u32(rtos->target,
			   rtos->symbols[CHROMIUM_EC_VAL_tasks].address +
			   params->task_offset_next * t,
			   &stack_ptr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to load TCB");
		return ret;
	}

	return rtos_generic_stack_read(rtos->target, params->stacking,
				       stack_ptr, reg_list, num_regs);
}

static int chromium_ec_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	size_t s;

	*symbol_list = calloc(ARRAY_SIZE(chromium_ec_symbol_list),
			      sizeof(symbol_table_elem_t));
	if (!(*symbol_list)) {
		LOG_ERROR("Chromium-EC: out of memory");
		return ERROR_FAIL;
	}

	for (s = 0; s < ARRAY_SIZE(chromium_ec_symbol_list); s++)
		(*symbol_list)[s].symbol_name = chromium_ec_symbol_list[s];

	return ERROR_OK;
}

const struct rtos_type chromium_ec_rtos = {
	.name = "Chromium-EC",
	.detect_rtos = chromium_ec_detect_rtos,
	.create = chromium_ec_create,
	.update_threads = chromium_ec_update_threads,
	.get_thread_reg_list = chromium_ec_get_thread_reg_list,
	.get_symbol_list_to_lookup = chromium_ec_get_symbol_list_to_lookup,
};
