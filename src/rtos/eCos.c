/***************************************************************************
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
#include "rtos_ecos_stackings.h"

static bool eCos_detect_rtos(struct target *target);
static int eCos_create(struct target *target);
static int eCos_update_threads(struct rtos *rtos);
static int eCos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id, struct rtos_reg **reg_list, int *num_regs);
static int eCos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[]);

struct eCos_thread_state {
	int value;
	const char *desc;
};

static const struct eCos_thread_state eCos_thread_states[] = {
	{ 0, "Ready" },
	{ 1, "Sleeping" },
	{ 2, "Countsleep" },
	{ 4, "Suspended" },
	{ 8, "Creating" },
	{ 16, "Exited" }
};

#define ECOS_NUM_STATES (sizeof(eCos_thread_states)/sizeof(struct eCos_thread_state))

struct eCos_params {
	const char *target_name;
	unsigned char pointer_width;
	unsigned char thread_stack_offset;
	unsigned char thread_name_offset;
	unsigned char thread_state_offset;
	unsigned char thread_next_offset;
	unsigned char thread_uniqueid_offset;
	const struct rtos_register_stacking *stacking_info;
};

static const struct eCos_params eCos_params_list[] = {
	{
	"cortex_m",			/* target_name */
	4,						/* pointer_width; */
	0x0c,					/* thread_stack_offset; */
	0x9c,					/* thread_name_offset; */
	0x3c,					/* thread_state_offset; */
	0xa0,					/* thread_next_offset */
	0x4c,					/* thread_uniqueid_offset */
	&rtos_eCos_Cortex_M3_stacking	/* stacking_info */
	}
};

#define ECOS_NUM_PARAMS ((int)(sizeof(eCos_params_list)/sizeof(struct eCos_params)))

enum eCos_symbol_values {
	eCos_VAL_thread_list = 0,
	eCos_VAL_current_thread_ptr = 1
};

static const char * const eCos_symbol_list[] = {
	"Cyg_Thread::thread_list",
	"Cyg_Scheduler_Base::current_thread",
	NULL
};

const struct rtos_type eCos_rtos = {
	.name = "eCos",

	.detect_rtos = eCos_detect_rtos,
	.create = eCos_create,
	.update_threads = eCos_update_threads,
	.get_thread_reg_list = eCos_get_thread_reg_list,
	.get_symbol_list_to_lookup = eCos_get_symbol_list_to_lookup,

};

static int eCos_update_threads(struct rtos *rtos)
{
	int retval;
	int tasks_found = 0;
	int thread_list_size = 0;
	const struct eCos_params *param;

	if (rtos == NULL)
		return -1;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (const struct eCos_params *) rtos->rtos_specific_params;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for eCos");
		return -4;
	}

	if (rtos->symbols[eCos_VAL_thread_list].address == 0) {
		LOG_ERROR("Don't have the thread list head");
		return -2;
	}

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* determine the number of current threads */
	uint32_t thread_list_head = rtos->symbols[eCos_VAL_thread_list].address;
	uint32_t thread_index;
	target_read_buffer(rtos->target,
		thread_list_head,
		param->pointer_width,
		(uint8_t *) &thread_index);
	uint32_t first_thread = thread_index;
	do {
		thread_list_size++;
		retval = target_read_buffer(rtos->target,
				thread_index + param->thread_next_offset,
				param->pointer_width,
				(uint8_t *) &thread_index);
		if (retval != ERROR_OK)
			return retval;
	} while (thread_index != first_thread);

	/* read the current thread id */
	uint32_t current_thread_addr;
	retval = target_read_buffer(rtos->target,
			rtos->symbols[eCos_VAL_current_thread_ptr].address,
			4,
			(uint8_t *)&current_thread_addr);
	if (retval != ERROR_OK)
		return retval;
	rtos->current_thread = 0;
	retval = target_read_buffer(rtos->target,
			current_thread_addr + param->thread_uniqueid_offset,
			2,
			(uint8_t *)&rtos->current_thread);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read eCos current thread from target");
		return retval;
	}

	if ((thread_list_size  == 0) || (rtos->current_thread == 0)) {
		/* Either : No RTOS threads - there is always at least the current execution though */
		/* OR     : No current thread - all threads suspended - show the current execution
		 * of idling */
		char tmp_str[] = "Current Execution";
		thread_list_size++;
		tasks_found++;
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * thread_list_size);
		rtos->thread_details->threadid = 1;
		rtos->thread_details->exists = true;
		rtos->thread_details->extra_info_str = NULL;
		rtos->thread_details->thread_name_str = malloc(sizeof(tmp_str));
		strcpy(rtos->thread_details->thread_name_str, tmp_str);

		if (thread_list_size == 0) {
			rtos->thread_count = 1;
			return ERROR_OK;
		}
	} else {
		/* create space for new thread details */
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * thread_list_size);
	}

	/* loop over all threads */
	thread_index = first_thread;
	do {

		#define ECOS_THREAD_NAME_STR_SIZE (200)
		char tmp_str[ECOS_THREAD_NAME_STR_SIZE];
		unsigned int i = 0;
		uint32_t name_ptr = 0;
		uint32_t prev_thread_ptr;

		/* Save the thread pointer */
		uint16_t thread_id;
		retval = target_read_buffer(rtos->target,
				thread_index + param->thread_uniqueid_offset,
				2,
				(uint8_t *)&thread_id);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read eCos thread id from target");
			return retval;
		}
		rtos->thread_details[tasks_found].threadid = thread_id;

		/* read the name pointer */
		retval = target_read_buffer(rtos->target,
				thread_index + param->thread_name_offset,
				param->pointer_width,
				(uint8_t *)&name_ptr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read eCos thread name pointer from target");
			return retval;
		}

		/* Read the thread name */
		retval =
			target_read_buffer(rtos->target,
				name_ptr,
				ECOS_THREAD_NAME_STR_SIZE,
				(uint8_t *)&tmp_str);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading thread name from eCos target");
			return retval;
		}
		tmp_str[ECOS_THREAD_NAME_STR_SIZE-1] = '\x00';

		if (tmp_str[0] == '\x00')
			strcpy(tmp_str, "No Name");

		rtos->thread_details[tasks_found].thread_name_str =
			malloc(strlen(tmp_str)+1);
		strcpy(rtos->thread_details[tasks_found].thread_name_str, tmp_str);

		/* Read the thread status */
		int64_t thread_status = 0;
		retval = target_read_buffer(rtos->target,
				thread_index + param->thread_state_offset,
				4,
				(uint8_t *)&thread_status);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading thread state from eCos target");
			return retval;
		}

		for (i = 0; (i < ECOS_NUM_STATES) && (eCos_thread_states[i].value != thread_status); i++) {
			/*
			 * empty
			 */
		}

		const char *state_desc;
		if  (i < ECOS_NUM_STATES)
			state_desc = eCos_thread_states[i].desc;
		else
			state_desc = "Unknown state";

		rtos->thread_details[tasks_found].extra_info_str = malloc(strlen(
					state_desc)+8);
		sprintf(rtos->thread_details[tasks_found].extra_info_str, "State: %s", state_desc);

		rtos->thread_details[tasks_found].exists = true;

		tasks_found++;
		prev_thread_ptr = thread_index;

		/* Get the location of the next thread structure. */
		thread_index = rtos->symbols[eCos_VAL_thread_list].address;
		retval = target_read_buffer(rtos->target,
				prev_thread_ptr + param->thread_next_offset,
				param->pointer_width,
				(uint8_t *) &thread_index);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading next thread pointer in eCos thread list");
			return retval;
		}
	} while (thread_index != first_thread);

	rtos->thread_count = tasks_found;
	return 0;
}

static int eCos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct eCos_params *param;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (const struct eCos_params *) rtos->rtos_specific_params;

	/* Find the thread with that thread id */
	uint16_t id = 0;
	uint32_t thread_list_head = rtos->symbols[eCos_VAL_thread_list].address;
	uint32_t thread_index;
	target_read_buffer(rtos->target, thread_list_head, param->pointer_width,
			(uint8_t *)&thread_index);
	bool done = false;
	while (!done) {
		retval = target_read_buffer(rtos->target,
				thread_index + param->thread_uniqueid_offset,
				2,
				(uint8_t *)&id);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading unique id from eCos thread");
			return retval;
		}

		if (id == thread_id) {
			done = true;
			break;
		}
		target_read_buffer(rtos->target,
			thread_index + param->thread_next_offset,
			param->pointer_width,
			(uint8_t *) &thread_index);
	}

	if (done) {
		/* Read the stack pointer */
		int64_t stack_ptr = 0;
		retval = target_read_buffer(rtos->target,
				thread_index + param->thread_stack_offset,
				param->pointer_width,
				(uint8_t *)&stack_ptr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading stack frame from eCos thread");
			return retval;
		}

		return rtos_generic_stack_read(rtos->target,
			param->stacking_info,
			stack_ptr,
			reg_list,
			num_regs);
	}

	return -1;
}

static int eCos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(eCos_symbol_list), sizeof(struct symbol_table_elem));

	for (i = 0; i < ARRAY_SIZE(eCos_symbol_list); i++)
		(*symbol_list)[i].symbol_name = eCos_symbol_list[i];

	return 0;
}

static bool eCos_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols != NULL) &&
			(target->rtos->symbols[eCos_VAL_thread_list].address != 0)) {
		/* looks like eCos */
		return true;
	}
	return false;
}

static int eCos_create(struct target *target)
{
	int i = 0;
	while ((i < ECOS_NUM_PARAMS) &&
		(0 != strcmp(eCos_params_list[i].target_name, target->type->name))) {
		i++;
	}
	if (i >= ECOS_NUM_PARAMS) {
		LOG_ERROR("Could not find target in eCos compatibility list");
		return -1;
	}

	target->rtos->rtos_specific_params = (void *) &eCos_params_list[i];
	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;
	return 0;
}
