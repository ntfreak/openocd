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
#include "target/register.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "server/gdb_server.h"

static bool hwthread_detect_rtos(struct target *target);
static int hwthread_create(struct target *target);
static int hwthread_update_threads(struct rtos *rtos);
static int hwthread_get_thread_reg(struct rtos *rtos, int64_t thread_id,
		uint32_t reg_num, struct rtos_reg *rtos_reg);
static int hwthread_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int hwthread_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);
static int hwthread_smp_init(struct target *target);
static int hwthread_set_reg(struct rtos *rtos, uint32_t reg_num, uint8_t *reg_value);

#define HW_THREAD_NAME_STR_SIZE (32)

extern int rtos_thread_packet(struct connection *connection, const char *packet, int packet_size);

static inline threadid_t threadid_from_target(const struct target *target)
{
	return target->coreid + 1;
}

const struct rtos_type hwthread_rtos = {
	.name = "hwthread",
	.detect_rtos = hwthread_detect_rtos,
	.create = hwthread_create,
	.update_threads = hwthread_update_threads,
	.get_thread_reg_list = hwthread_get_thread_reg_list,
	.get_thread_reg = hwthread_get_thread_reg,
	.get_symbol_list_to_lookup = hwthread_get_symbol_list_to_lookup,
	.smp_init = hwthread_smp_init,
	.set_reg = hwthread_set_reg,
};

struct hwthread_params {
	int dummy_param;
};

static int hwthread_fill_thread(struct rtos *rtos, struct target *curr, int thread_num)
{
	char tmp_str[HW_THREAD_NAME_STR_SIZE];
	threadid_t tid = threadid_from_target(curr);

	memset(tmp_str, 0, HW_THREAD_NAME_STR_SIZE);

	/* thread-id is the core-id of this core inside the SMP group plus 1 */
	rtos->thread_details[thread_num].threadid = tid;
	/* create the thread name */
	rtos->thread_details[thread_num].exists = true;
	rtos->thread_details[thread_num].thread_name_str = strdup(target_name(curr));
	snprintf(tmp_str, HW_THREAD_NAME_STR_SIZE-1, "state: %s", debug_reason_name(curr));
	rtos->thread_details[thread_num].extra_info_str = strdup(tmp_str);

	return ERROR_OK;
}

static int hwthread_update_threads(struct rtos *rtos)
{
	int threads_found = 0;
	int thread_list_size = 0;
	struct target_list *head;
	struct target *target;
	int64_t current_thread = 0;
	enum target_debug_reason current_reason = DBG_REASON_UNDEFINED;

	if (rtos == NULL)
		return -1;

	target = rtos->target;

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* determine the number of "threads" */
	if (target->smp) {
		for (head = target->head; head != NULL; head = head->next) {
			struct target *curr = head->target;

			if (!target_was_examined(curr))
				continue;

			++thread_list_size;
		}
	} else
		thread_list_size = 1;

	/* create space for new thread details */
	rtos->thread_details = malloc(sizeof(struct thread_detail) * thread_list_size);

	if (target->smp) {
		/* loop over all threads */
		for (head = target->head; head != NULL; head = head->next) {
			struct target *curr = head->target;

			if (!target_was_examined(curr))
				continue;

			threadid_t tid = threadid_from_target(curr);

			hwthread_fill_thread(rtos, curr, threads_found);

			/* find an interesting thread to set as current */
			switch (current_reason) {
			case DBG_REASON_UNDEFINED:
				current_reason = curr->debug_reason;
				current_thread = tid;
				break;
			case DBG_REASON_SINGLESTEP:
				/* single-step can only be overridden by itself */
				if (curr->debug_reason == DBG_REASON_SINGLESTEP) {
					if (tid == rtos->current_threadid)
						current_thread = tid;
				}
				break;
			case DBG_REASON_BREAKPOINT:
				/* single-step overrides breakpoint */
				if (curr->debug_reason == DBG_REASON_SINGLESTEP) {
					current_reason = curr->debug_reason;
					current_thread = tid;
				} else
				/* multiple breakpoints, prefer gdbs' threadid */
				if (curr->debug_reason == DBG_REASON_BREAKPOINT) {
					if (tid == rtos->current_threadid)
						current_thread = tid;
				}
				break;
			case DBG_REASON_WATCHPOINT:
				/* breakpoint and single-step override watchpoint */
				if (curr->debug_reason == DBG_REASON_SINGLESTEP ||
						curr->debug_reason == DBG_REASON_BREAKPOINT) {
					current_reason = curr->debug_reason;
					current_thread = tid;
				}
				break;
			case DBG_REASON_DBGRQ:
				/* all other reasons override debug-request */
				if (curr->debug_reason == DBG_REASON_SINGLESTEP ||
						curr->debug_reason == DBG_REASON_WATCHPOINT ||
						curr->debug_reason == DBG_REASON_BREAKPOINT) {
					current_reason = curr->debug_reason;
					current_thread = tid;
				} else
				if (curr->debug_reason == DBG_REASON_DBGRQ) {
					if (tid == rtos->current_threadid)
						current_thread = tid;
				}

				break;

			default:
				break;
			}

			threads_found++;
		}
	} else {
		hwthread_fill_thread(rtos, target, threads_found);
		current_thread = threadid_from_target(target);
		threads_found++;
	}

	rtos->thread_count = threads_found;

	/* we found an interesting thread, set it as current */
	if (current_thread != 0)
		rtos->current_thread = current_thread;
	else if (rtos->current_threadid != 0)
		rtos->current_thread = rtos->current_threadid;
	else
		rtos->current_thread = threadid_from_target(target);

	LOG_DEBUG("%s current_thread=%i", __func__, (int)rtos->current_thread);
	return 0;
}

static int hwthread_smp_init(struct target *target)
{
	return hwthread_update_threads(target->rtos);
}

static struct target *hwthread_find_thread(struct target *target, int64_t thread_id)
{
	/* Find the thread with that thread_id */
	if (target == NULL)
		return NULL;
	if (target->smp) {
		for (struct target_list *head = target->head; head != NULL; head = head->next) {
			if (thread_id == threadid_from_target(head->target))
				return head->target;
		}
	} else if (thread_id == threadid_from_target(target)) {
		return target;
	}
	return NULL;
}

static int hwthread_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **rtos_reg_list, int *rtos_reg_list_size)
{
	if (rtos == NULL)
		return ERROR_FAIL;

	struct target *target = rtos->target;

	struct target *curr = hwthread_find_thread(target, thread_id);
	if (curr == NULL)
		return ERROR_FAIL;

	if (!target_was_examined(curr))
		return ERROR_FAIL;

	int reg_list_size;
	struct reg **reg_list;
	int retval = target_get_gdb_reg_list(curr, &reg_list, &reg_list_size,
			REG_CLASS_GENERAL);
	if (retval != ERROR_OK)
		return retval;

	int j = 0;
	for (int i = 0; i < reg_list_size; i++) {
		if (reg_list[i] == NULL || reg_list[i]->exist == false || reg_list[i]->hidden)
			continue;
		j++;
	}
	*rtos_reg_list_size = j;
	*rtos_reg_list = calloc(*rtos_reg_list_size, sizeof(struct rtos_reg));
	if (*rtos_reg_list == NULL) {
		free(reg_list);
		return ERROR_FAIL;
	}

	j = 0;
	for (int i = 0; i < reg_list_size; i++) {
		if (reg_list[i] == NULL || reg_list[i]->exist == false || reg_list[i]->hidden)
			continue;
		(*rtos_reg_list)[j].number = (*reg_list)[i].number;
		(*rtos_reg_list)[j].size = (*reg_list)[i].size;
		memcpy((*rtos_reg_list)[j].value, (*reg_list)[i].value,
				((*reg_list)[i].size + 7) / 8);
		j++;
	}
	free(reg_list);

	return ERROR_OK;
}

static int hwthread_get_thread_reg(struct rtos *rtos, int64_t thread_id,
		uint32_t reg_num, struct rtos_reg *rtos_reg)
{
	if (rtos == NULL)
		return ERROR_FAIL;

	struct target *target = rtos->target;

	struct target *curr = hwthread_find_thread(target, thread_id);
	if (curr == NULL) {
		LOG_ERROR("Couldn't find RTOS thread for id %" PRId64 ".", thread_id);
		return ERROR_FAIL;
	}

	if (!target_was_examined(curr)) {
		LOG_ERROR("Target %d hasn't been examined yet.", curr->coreid);
		return ERROR_FAIL;
	}

	struct reg *reg = register_get_by_number(curr->reg_cache, reg_num, true);
	if (!reg) {
		LOG_ERROR("Couldn't find register %" PRIu32 " in thread %" PRId64 ".", reg_num,
				thread_id);
		return ERROR_FAIL;
	}

	if (reg->type->get(reg) != ERROR_OK)
		return ERROR_FAIL;

	rtos_reg->number = reg->number;
	rtos_reg->size = reg->size;
	unsigned bytes = (reg->size + 7) / 8;
	assert(bytes <= sizeof(rtos_reg->value));
	memcpy(rtos_reg->value, reg->value, bytes);

	return ERROR_OK;
}

static int hwthread_set_reg(struct rtos *rtos, uint32_t reg_num, uint8_t *reg_value)
{
	if (rtos == NULL)
		return ERROR_FAIL;

	struct target *target = rtos->target;

	struct target *curr = hwthread_find_thread(target, rtos->current_thread);
	if (curr == NULL)
		return ERROR_FAIL;

	struct reg *reg = register_get_by_number(curr->reg_cache, reg_num, true);
	if (!reg)
		return ERROR_FAIL;

	return reg->type->set(reg, reg_value);
}

static int hwthread_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	/* return an empty list, we don't have any symbols to look up */
	*symbol_list = calloc(1, sizeof(symbol_table_elem_t));
	(*symbol_list)[0].symbol_name = NULL;
	return 0;
}

static int hwthread_target_for_threadid(struct connection *connection, int64_t thread_id, struct target **p_target)
{
	struct target *target = get_target_from_connection(connection);

	struct target *curr = hwthread_find_thread(target, thread_id);
	if (curr == NULL)
		return ERROR_FAIL;

	*p_target = curr;

	return ERROR_OK;
}

static bool hwthread_detect_rtos(struct target *target)
{
	/* always return 0, avoid auto-detection */
	return false;
}

static int hwthread_thread_packet(struct connection *connection, const char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);

	struct target *curr = NULL;
	int64_t current_threadid;

	if (packet[0] == 'H' && packet[1] == 'g') {
		sscanf(packet, "Hg%16" SCNx64, &current_threadid);

		if (current_threadid > 0) {
			if (hwthread_target_for_threadid(connection, current_threadid, &curr) != ERROR_OK) {
				LOG_ERROR("hwthread: cannot find thread id %"PRId64, current_threadid);
				gdb_put_packet(connection, "E01", 3);
				return ERROR_FAIL;
			}
			target->rtos->current_thread = current_threadid;
		} else
		if (current_threadid == 0 || current_threadid == -1)
			target->rtos->current_thread = threadid_from_target(target);

		target->rtos->current_threadid = current_threadid;

		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}

	return rtos_thread_packet(connection, packet, packet_size);
}

static int hwthread_create(struct target *target)
{
	LOG_INFO("Hardware thread awareness created");

	target->rtos->rtos_specific_params = NULL;
	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;
	target->rtos->gdb_target_for_threadid = hwthread_target_for_threadid;
	target->rtos->gdb_thread_packet = hwthread_thread_packet;
	return 0;
}
