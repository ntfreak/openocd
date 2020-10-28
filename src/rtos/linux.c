/***************************************************************************
 *   Copyright (C) 2011 by STEricsson                                      *
 *   Heythem Bouhaja heythem.bouhaja@stericsson.com   : creation           *
 *   Michel JAOUEN michel.jaouen@stericsson.com : adaptation to rtos       *
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
#include "helper/log.h"
#include "helper/types.h"
#include "rtos.h"
#include "rtos_standard_stackings.h"
#include <target/register.h>
#include "server/gdb_server.h"

#define LINUX_USER_KERNEL_BORDER 0xc0000000
#include "linux_header.h"
#define PHYS
#define MAX_THREADS 200
/*  specific task  */
struct linux_os {
	const char *name;
	uint32_t init_task_addr;
	int thread_count;
	int threadid_count;
	int preupdtate_threadid_count;
	int nr_cpus;
	int threads_lookup;
	int threads_needs_update;
	struct current_thread *current_threads;
	struct threads *thread_list;
	/*  virt2phys parameter */
	uint32_t phys_mask;
	uint32_t phys_base;
};

struct current_thread {
	int64_t threadid;
	int32_t core_id;
#ifdef PID_CHECK
	uint32_t pid;
#endif
	uint32_t TS;
	struct current_thread *next;
};

struct threads {
	char name[17];
	uint32_t base_addr;	/*  address to read magic */
	uint32_t state;		/*  magic value : filled only at creation */
	uint32_t pid;		/* linux pid : id for identifying a thread */
	uint32_t oncpu;		/* content cpu number in current thread */
	uint32_t asid;		/*  filled only at creation  */
	int64_t threadid;
	int status;		/* dead = 1 alive = 2 current = 3 alive and current */
	/*  value that should not change during the live of a thread ? */
	uint32_t thread_info_addr;	/*  contain latest thread_info_addr computed */
	/*  retrieve from thread_info */
	struct cpu_context *context;
	struct threads *next;
};

struct cpu_context {
	uint32_t R4;
	uint32_t R5;
	uint32_t R6;
	uint32_t R7;
	uint32_t R8;
	uint32_t R9;
	uint32_t IP;
	uint32_t FP;
	uint32_t SP;
	uint32_t PC;
	uint32_t preempt_count;
};
static struct cpu_context *cpu_context_read(struct target *target, uint32_t base_addr,
				     uint32_t *info_addr);
static int insert_into_threadlist(struct target *target, struct threads *t);

static int linux_os_create(struct target *target);

static int linux_os_dummy_update(struct rtos *rtos)
{
	/*  update is done only when thread request come
	 *  too many thread to do it on each stop */
	return 0;
}

static int linux_compute_virt2phys(struct target *target, target_addr_t address)
{
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	target_addr_t pa = 0;
	int retval = target->type->virt2phys(target, address, &pa);
	if (retval != ERROR_OK) {
		LOG_ERROR("Cannot compute linux virt2phys translation");
		/*  fixes default address  */
		linux_os->phys_base = 0;
		return ERROR_FAIL;
	}

	linux_os->init_task_addr = address;
	address = address & linux_os->phys_mask;
	linux_os->phys_base = pa - address;
	return ERROR_OK;
}

static int linux_read_memory(struct target *target,
	uint32_t address, uint32_t size, uint32_t count,
	uint8_t *buffer)
{
#ifdef PHYS
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	uint32_t pa = (address & linux_os->phys_mask) + linux_os->phys_base;
#endif
	if (address < 0xc000000) {
		LOG_ERROR("linux awareness : address in user space");
		return ERROR_FAIL;
	}
#ifdef PHYS
	target_read_phys_memory(target, pa, size, count, buffer);
#endif
	target_read_memory(target, address, size, count, buffer);
	return ERROR_OK;
}

static int fill_buffer(struct target *target, uint32_t addr, uint8_t *buffer)
{

	if ((addr & 0xfffffffc) != addr)
		LOG_INFO("unaligned address %" PRIx32 "!!", addr);

	int retval = linux_read_memory(target, addr, 4, 1, buffer);
	return retval;

}

static uint32_t get_buffer(struct target *target, const uint8_t *buffer)
{
	uint32_t value = 0;
	const uint8_t *value_ptr = buffer;
	value = target_buffer_get_u32(target, value_ptr);
	return value;
}

static int linux_os_thread_reg_list(struct rtos *rtos,
	int64_t thread_id, struct rtos_reg **reg_list, int *num_regs)
{
	struct target *target = rtos->target;
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	struct current_thread *tmp = linux_os->current_threads;
	struct current_thread *next;
	int found = 0;
	int retval;
	/*  check if a current thread is requested  */
	next = tmp;

	do {
		if (next->threadid == thread_id)
			found = 1;
		else
			next = next->next;
	} while ((found == 0) && (next != tmp) && (next != NULL));

	if (found == 0) {
		LOG_ERROR("could not find thread: %" PRIx64, thread_id);
		return ERROR_FAIL;
	}

	/*  search target to perform the access  */
	struct reg **gdb_reg_list;
	struct target_list *head;
	head = target->head;
	found = 0;
	do {
		if (head->target->coreid == next->core_id) {

			target = head->target;
			found = 1;
		} else
			head = head->next;

	} while ((head != (struct target_list *)NULL) && (found == 0));

	if (found == 0) {
		LOG_ERROR
		(
			"current thread %" PRIx64 ": no target to perform access of core id %" PRIx32,
			thread_id,
			next->core_id);
		return ERROR_FAIL;
	}

	/*LOG_INFO("thread %lx current on core %x",thread_id, target->coreid);*/
	retval = target_get_gdb_reg_list(target, &gdb_reg_list, num_regs, REG_CLASS_GENERAL);
	if (retval != ERROR_OK)
		return retval;

	*reg_list = calloc(*num_regs, sizeof(struct rtos_reg));

	for (int i = 0; i < *num_regs; ++i) {
		if (!gdb_reg_list[i]->valid)
			gdb_reg_list[i]->type->get(gdb_reg_list[i]);

		(*reg_list)[i].number = gdb_reg_list[i]->number;
		(*reg_list)[i].size = gdb_reg_list[i]->size;

		buf_cpy(gdb_reg_list[i]->value, (*reg_list)[i].value, (*reg_list)[i].size);
	}

	return ERROR_OK;
}

static bool linux_os_detect(struct target *target)
{
	LOG_INFO("should no be called");
	return false;
}

static int linux_os_smp_init(struct target *target);
static int linux_os_clean(struct target *target);
#define INIT_TASK 0
static const char * const linux_symbol_list[] = {
	"init_task",
	NULL
};

static int linux_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;
	*symbol_list = (symbol_table_elem_t *)
		calloc(ARRAY_SIZE(linux_symbol_list), sizeof(symbol_table_elem_t));

	for (i = 0; i < ARRAY_SIZE(linux_symbol_list); i++)
		(*symbol_list)[i].symbol_name = linux_symbol_list[i];

	return 0;
}

static char *linux_ps_command(struct target *target);

const struct rtos_type Linux_os = {
	.name = "linux",
	.detect_rtos = linux_os_detect,
	.create = linux_os_create,
	.smp_init = linux_os_smp_init,
	.update_threads = linux_os_dummy_update,
	.get_thread_reg_list = linux_os_thread_reg_list,
	.get_symbol_list_to_lookup = linux_get_symbol_list_to_lookup,
	.clean = linux_os_clean,
	.ps_command = linux_ps_command,
};

static int linux_thread_packet(struct connection *connection, char const *packet,
		int packet_size);
static void linux_identify_current_threads(struct target *target);

#ifdef PID_CHECK
int fill_task_pid(struct target *target, struct threads *t)
{
	uint32_t pid_addr = t->base_addr + PID;
	uint8_t buffer[4];
	int retval = fill_buffer(target, pid_addr, buffer);

	if (retval == ERROR_OK) {
		uint32_t val = get_buffer(target, buffer);
		t->pid = val;
	} else
		LOG_ERROR("fill_task_pid: unable to read memory");

	return retval;
}
#endif

static int fill_task(struct target *target, struct threads *t)
{
	int retval;
	uint32_t pid_addr = t->base_addr + PID;
	uint32_t mem_addr = t->base_addr + MEM;
	uint32_t on_cpu = t->base_addr + ONCPU;
	uint8_t *buffer = calloc(1, 4);
	retval = fill_buffer(target, t->base_addr, buffer);

	if (retval == ERROR_OK) {
		uint32_t val = get_buffer(target, buffer);
		t->state = val;
	} else
		LOG_ERROR("fill_task: unable to read memory");

	retval = fill_buffer(target, pid_addr, buffer);

	if (retval == ERROR_OK) {
		uint32_t val = get_buffer(target, buffer);
		t->pid = val;
	} else
		LOG_ERROR("fill task: unable to read memory");

	retval = fill_buffer(target, on_cpu, buffer);

	if (retval == ERROR_OK) {
		uint32_t val = get_buffer(target, buffer);
		t->oncpu = val;
	} else
		LOG_ERROR("fill task: unable to read memory");

	retval = fill_buffer(target, mem_addr, buffer);

	if (retval == ERROR_OK) {
		uint32_t val = get_buffer(target, buffer);

		if (val != 0) {
			uint32_t asid_addr = val + MM_CTX;
			retval = fill_buffer(target, asid_addr, buffer);

			if (retval == ERROR_OK) {
				val = get_buffer(target, buffer);
				t->asid = val;
			} else
				LOG_ERROR
					("fill task: unable to read memory -- ASID");
		} else
			t->asid = 0;
	} else
		LOG_ERROR("fill task: unable to read memory");

	free(buffer);

	return retval;
}

static int get_name(struct target *target, struct threads *t)
{
	int retval;
	uint32_t full_name[4];
	uint32_t comm = t->base_addr + COMM;
	int i;

	for (i = 0; i < 17; i++)
		t->name[i] = 0;

	retval = linux_read_memory(target, comm, 4, 4, (uint8_t *) full_name);

	if (retval != ERROR_OK) {
		LOG_ERROR("get_name: unable to read memory\n");
		return ERROR_FAIL;
	}

	uint32_t raw_name = target_buffer_get_u32(target,
			(const uint8_t *)
			&full_name[0]);
	t->name[3] = raw_name >> 24;
	t->name[2] = raw_name >> 16;
	t->name[1] = raw_name >> 8;
	t->name[0] = raw_name;
	raw_name =
		target_buffer_get_u32(target, (const uint8_t *)&full_name[1]);
	t->name[7] = raw_name >> 24;
	t->name[6] = raw_name >> 16;
	t->name[5] = raw_name >> 8;
	t->name[4] = raw_name;
	raw_name =
		target_buffer_get_u32(target, (const uint8_t *)&full_name[2]);
	t->name[11] = raw_name >> 24;
	t->name[10] = raw_name >> 16;
	t->name[9] = raw_name >> 8;
	t->name[8] = raw_name;
	raw_name =
		target_buffer_get_u32(target, (const uint8_t *)&full_name[3]);
	t->name[15] = raw_name >> 24;
	t->name[14] = raw_name >> 16;
	t->name[13] = raw_name >> 8;
	t->name[12] = raw_name;
	return ERROR_OK;

}

static int get_current(struct target *target, int create)
{
	struct target_list *head;
	head = target->head;
	uint8_t *buf;
	uint32_t val;
	uint32_t ti_addr;
	uint8_t *buffer = calloc(1, 4);
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	struct current_thread *ctt = linux_os->current_threads;

	/*  invalid current threads content */
	while (ctt != NULL) {
		ctt->threadid = -1;
		ctt->TS = 0xdeadbeef;
		ctt = ctt->next;
	}

	while (head != (struct target_list *)NULL) {
		struct reg **reg_list;
		int reg_list_size;
		int retval;

		if (target_get_gdb_reg_list(head->target, &reg_list,
				&reg_list_size, REG_CLASS_GENERAL) != ERROR_OK) {
			free(buffer);
			return ERROR_TARGET_FAILURE;
		}

		if (!reg_list[13]->valid)
			reg_list[13]->type->get(reg_list[13]);

		buf = reg_list[13]->value;
		val = get_buffer(target, buf);
		ti_addr = (val & 0xffffe000);
		uint32_t TS_addr = ti_addr + 0xc;
		retval = fill_buffer(target, TS_addr, buffer);

		if (retval == ERROR_OK) {
			uint32_t TS = get_buffer(target, buffer);
			uint32_t cpu, on_cpu = TS + ONCPU;
			retval = fill_buffer(target, on_cpu, buffer);

			if (retval == ERROR_OK) {
				/*uint32_t cpu = get_buffer(target, buffer);*/
				struct current_thread *ct =
					linux_os->current_threads;
				cpu = head->target->coreid;

				while ((ct != NULL) && (ct->core_id != (int32_t) cpu))
					ct = ct->next;

				if ((ct != NULL) && (ct->TS == 0xdeadbeef))
					ct->TS = TS;
				else
					LOG_ERROR
						("error in linux current thread update");

				if (create && ct) {
					struct threads *t;
					t = calloc(1, sizeof(struct threads));
					t->base_addr = ct->TS;
					fill_task(target, t);
					get_name(target, t);
					t->oncpu = cpu;
					insert_into_threadlist(target, t);
					t->status = 3;
					t->thread_info_addr = 0xdeadbeef;
					ct->threadid = t->threadid;
					linux_os->thread_count++;
#ifdef PID_CHECK
					ct->pid = t->pid;
#endif
					/*LOG_INFO("Creation of current thread %s",t->name);*/
				}
			}
		}

		free(reg_list);
		head = head->next;
	}

	free(buffer);

	return ERROR_OK;
}

static struct cpu_context *cpu_context_read(struct target *target, uint32_t base_addr,
	uint32_t *thread_info_addr_old)
{
	struct cpu_context *context = calloc(1, sizeof(struct cpu_context));
	uint32_t preempt_count_addr = 0;
	uint32_t registers[10];
	uint8_t *buffer = calloc(1, 4);
	uint32_t stack = base_addr + QAT;
	uint32_t thread_info_addr = 0;
	uint32_t thread_info_addr_update = 0;
	int retval = ERROR_FAIL;
	context->R4 = 0xdeadbeef;
	context->R5 = 0xdeadbeef;
	context->R6 = 0xdeadbeef;
	context->R7 = 0xdeadbeef;
	context->R8 = 0xdeadbeef;
	context->R9 = 0xdeadbeef;
	context->IP = 0xdeadbeef;
	context->FP = 0xdeadbeef;
	context->SP = 0xdeadbeef;
	context->PC = 0xdeadbeef;
retry:

	if (*thread_info_addr_old == 0xdeadbeef) {
		retval = fill_buffer(target, stack, buffer);

		if (retval == ERROR_OK)
			thread_info_addr = get_buffer(target, buffer);
		else
			LOG_ERROR("cpu_context: unable to read memory");

		thread_info_addr_update = thread_info_addr;
	} else
		thread_info_addr = *thread_info_addr_old;

	preempt_count_addr = thread_info_addr + PREEMPT;
	retval = fill_buffer(target, preempt_count_addr, buffer);

	if (retval == ERROR_OK)
		context->preempt_count = get_buffer(target, buffer);
	else {
		if (*thread_info_addr_old != 0xdeadbeef) {
			LOG_ERROR
				("cpu_context: cannot read at thread_info_addr");

			if (*thread_info_addr_old < LINUX_USER_KERNEL_BORDER)
				LOG_INFO
					("cpu_context : thread_info_addr in userspace!!!");

			*thread_info_addr_old = 0xdeadbeef;
			goto retry;
		}

		LOG_ERROR("cpu_context: unable to read memory");
	}

	thread_info_addr += CPU_CONT;

	retval = linux_read_memory(target, thread_info_addr, 4, 10,
			(uint8_t *) registers);

	if (retval != ERROR_OK) {
		free(buffer);
		LOG_ERROR("cpu_context: unable to read memory\n");
		return context;
	}

	context->R4 =
		target_buffer_get_u32(target, (const uint8_t *)&registers[0]);
	context->R5 =
		target_buffer_get_u32(target, (const uint8_t *)&registers[1]);
	context->R6 =
		target_buffer_get_u32(target, (const uint8_t *)&registers[2]);
	context->R7 =
		target_buffer_get_u32(target, (const uint8_t *)&registers[3]);
	context->R8 =
		target_buffer_get_u32(target, (const uint8_t *)&registers[4]);
	context->R9 =
		target_buffer_get_u32(target, (const uint8_t *)&registers[5]);
	context->IP =
		target_buffer_get_u32(target, (const uint8_t *)&registers[6]);
	context->FP =
		target_buffer_get_u32(target, (const uint8_t *)&registers[7]);
	context->SP =
		target_buffer_get_u32(target, (const uint8_t *)&registers[8]);
	context->PC =
		target_buffer_get_u32(target, (const uint8_t *)&registers[9]);

	if (*thread_info_addr_old == 0xdeadbeef)
		*thread_info_addr_old = thread_info_addr_update;

	free(buffer);

	return context;
}

static uint32_t next_task(struct target *target, struct threads *t)
{
	uint8_t *buffer = calloc(1, 4);
	uint32_t next_addr = t->base_addr + NEXT;
	int retval = fill_buffer(target, next_addr, buffer);

	if (retval == ERROR_OK) {
		uint32_t val = get_buffer(target, buffer);
		val = val - NEXT;
		free(buffer);
		return val;
	} else
		LOG_ERROR("next task: unable to read memory");

	free(buffer);

	return 0;
}

static struct current_thread *add_current_thread(struct current_thread *currents,
	struct current_thread *ct)
{
	ct->next = NULL;

	if (currents == NULL) {
		currents = ct;
		return currents;
	} else {
		struct current_thread *temp = currents;

		while (temp->next != NULL)
			temp = temp->next;

		temp->next = ct;
		return currents;
	}
}

static struct threads *liste_del_task(struct threads *task_list, struct threads **t,
	struct threads *prev)
{
	LOG_INFO("del task %" PRId64, (*t)->threadid);
	if (prev)
		prev->next = (*t)->next;
	else
		task_list = (*t)->next;

	/*  free content of threads */
	free((*t)->context);

	free(*t);
	*t = prev ? prev : task_list;
	return task_list;
}

static struct threads *liste_add_task(struct threads *task_list, struct threads *t,
	struct threads **last)
{
	t->next = NULL;

	if (*last == NULL)
		if (task_list == NULL) {
			task_list = t;
			return task_list;
		} else {
			struct threads *temp = task_list;

			while (temp->next != NULL)
				temp = temp->next;

			temp->next = t;
			*last = t;
			return task_list;
		} else {
		(*last)->next = t;
		*last = t;
		return task_list;
	}
}

#ifdef PID_CHECK
static int current_pid(struct linux_os *linux_os, uint32_t pid)
#else
static int current_base_addr(struct linux_os *linux_os, uint32_t base_addr)
#endif
{
	struct current_thread *ct = linux_os->current_threads;
#ifdef PID_CHECK

	while ((ct != NULL) && (ct->pid != pid))
#else
	while ((ct != NULL) && (ct->TS != base_addr))
#endif
		ct = ct->next;
#ifdef PID_CHECK
	if ((ct != NULL) && (ct->pid == pid))
#else
	if ((ct != NULL) && (ct->TS == base_addr))
#endif
		return 1;

	return 0;
}

static int linux_get_tasks(struct target *target, int context)
{
	int loop = 0;
	int retval = 0;
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	linux_os->thread_list = NULL;
	linux_os->thread_count = 0;

	if (linux_os->init_task_addr == 0xdeadbeef) {
		LOG_INFO("no init symbol\n");
		return ERROR_FAIL;
	}

	int64_t start = timeval_ms();

	struct threads *t = calloc(1, sizeof(struct threads));
	struct threads *last = NULL;
	t->base_addr = linux_os->init_task_addr;
	/* retrieve the thread id , currently running in the different smp core */
	get_current(target, 1);

	while (((t->base_addr != linux_os->init_task_addr) &&
		(t->base_addr != 0)) || (loop == 0)) {
		loop++;
		fill_task(target, t);
		retval = get_name(target, t);

		if (loop > MAX_THREADS) {
			free(t);
			LOG_INFO("more than %d threads !!", MAX_THREADS);
			return ERROR_FAIL;
		}

		if (retval != ERROR_OK) {
			free(t);
			return ERROR_FAIL;
		}

		/*  check that this thread is not one the current threads already
		 *  created */
		uint32_t base_addr;
#ifdef PID_CHECK

		if (!current_pid(linux_os, t->pid)) {
#else
		if (!current_base_addr(linux_os, t->base_addr)) {
#endif
			t->threadid = linux_os->threadid_count;
			t->status = 1;
			linux_os->threadid_count++;

			linux_os->thread_list =
				liste_add_task(linux_os->thread_list, t, &last);
			/* no interest to fill the context if it is a current thread. */
			linux_os->thread_count++;
			t->thread_info_addr = 0xdeadbeef;

			if (context)
				t->context =
					cpu_context_read(target, t->base_addr,
						&t->thread_info_addr);
			base_addr = next_task(target, t);
		} else {
			/*LOG_INFO("thread %s is a current thread already created",t->name); */
			base_addr = next_task(target, t);
			free(t);
		}

		t = calloc(1, sizeof(struct threads));
		t->base_addr = base_addr;
	}

	linux_os->threads_lookup = 1;
	linux_os->threads_needs_update = 0;
	linux_os->preupdtate_threadid_count = linux_os->threadid_count - 1;
	/*  check that all current threads have been identified  */

	LOG_INFO("complete time %" PRId64 ", thread mean %" PRId64 "\n",
		(timeval_ms() - start),
		(timeval_ms() - start) / linux_os->threadid_count);

	LOG_INFO("threadid count %d", linux_os->threadid_count);
	free(t);

	return ERROR_OK;
}

static int clean_threadlist(struct target *target)
{
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	struct threads *old, *temp = linux_os->thread_list;

	while (temp != NULL) {
		old = temp;

		free(temp->context);

		temp = temp->next;
		free(old);
	}

	return ERROR_OK;
}

static int linux_os_clean(struct target *target)
{
	struct linux_os *os_linux = (struct linux_os *)
		target->rtos->rtos_specific_params;
	clean_threadlist(target);
	os_linux->init_task_addr = 0xdeadbeef;
	os_linux->name = "linux";
	os_linux->thread_list = NULL;
	os_linux->thread_count = 0;
	os_linux->nr_cpus = 0;
	os_linux->threads_lookup = 0;
	os_linux->threads_needs_update = 0;
	os_linux->threadid_count = 1;
	return ERROR_OK;
}

static int insert_into_threadlist(struct target *target, struct threads *t)
{
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	struct threads *temp = linux_os->thread_list;
	t->threadid = linux_os->threadid_count;
	linux_os->threadid_count++;
	t->status = 1;
	t->next = NULL;

	if (temp == NULL)
		linux_os->thread_list = t;
	else {
		while (temp->next != NULL)
			temp = temp->next;

		t->next = NULL;
		temp->next = t;
	}

	return ERROR_OK;
}

static void linux_identify_current_threads(struct target *target)
{
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	struct threads *thread_list = linux_os->thread_list;
	struct current_thread *ct = linux_os->current_threads;
	struct threads *t = NULL;

	while ((ct != NULL)) {
		if (ct->threadid == -1) {

			/*  un-identified thread */
			int found = 0;
			t = calloc(1, sizeof(struct threads));
			t->base_addr = ct->TS;
#ifdef PID_CHECK

			if (fill_task_pid(target, t) != ERROR_OK) {
error_handling:
				free(t);
				LOG_ERROR
					("linux identify_current_threads: unable to read pid");
				return;
			}
#endif

			/* search in the list of threads if pid
			   already present */
			while ((thread_list != NULL) && (found == 0)) {
#ifdef PID_CHECK
				if (thread_list->pid == t->pid) {
#else
				if (thread_list->base_addr == t->base_addr) {
#endif
					free(t);
					t = thread_list;
					found = 1;
				}
				thread_list = thread_list->next;
			}

			if (!found) {
				/*  it is a new thread */
				if (fill_task(target, t) != ERROR_OK)
					goto error_handling;

				get_name(target, t);
				insert_into_threadlist(target, t);
				t->thread_info_addr = 0xdeadbeef;
			}

			t->status = 3;
			ct->threadid = t->threadid;
#ifdef PID_CHECK
			ct->pid = t->pid;
#endif
			linux_os->thread_count++;
#if 0
			if (found == 0)
				LOG_INFO("current thread core %x identified %s",
					ct->core_id, t->name);
			else
				LOG_INFO("current thread core %x, reused %s",
					ct->core_id, t->name);
#endif
		}
#if 0
		else {
			struct threads tmp;
			tmp.base_addr = ct->TS;
			get_name(target, &tmp);
			LOG_INFO("current thread core %x , already identified %s !!!",
				ct->core_id, tmp.name);
		}
#endif
		ct = ct->next;
	}

	return;
#ifndef PID_CHECK
error_handling:
	free(t);
	LOG_ERROR("unable to read pid");
	return;

#endif
}

static int linux_task_update(struct target *target, int context)
{
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	struct threads *thread_list = linux_os->thread_list;
	int retval;
	int loop = 0;
	linux_os->thread_count = 0;

	/*thread_list = thread_list->next; skip init_task*/
	while (thread_list != NULL) {
		thread_list->status = 0;	/*setting all tasks to dead state*/

		free(thread_list->context);
		thread_list->context = NULL;

		thread_list = thread_list->next;
	}

	int found = 0;

	if (linux_os->init_task_addr == 0xdeadbeef) {
		LOG_INFO("no init symbol\n");
		return ERROR_FAIL;
	}
	int64_t start = timeval_ms();
	struct threads *t = calloc(1, sizeof(struct threads));
	uint32_t previous = 0xdeadbeef;
	t->base_addr = linux_os->init_task_addr;
	retval = get_current(target, 0);
	/*check that all current threads have been identified  */
	linux_identify_current_threads(target);

	while (((t->base_addr != linux_os->init_task_addr) &&
		(t->base_addr != previous)) || (loop == 0)) {
		/*  for avoiding any permanent loop for any reason possibly due to
		 *  target */
		loop++;
		previous = t->base_addr;
		/*  read only pid */
#ifdef PID_CHECK
		retval = fill_task_pid(target, t);
#endif

		if (retval != ERROR_OK) {
			free(t);
			return ERROR_FAIL;
		}

		thread_list = linux_os->thread_list;

		while (thread_list != NULL) {
#ifdef PID_CHECK
			if (t->pid == thread_list->pid) {
#else
			if (t->base_addr == thread_list->base_addr) {
#endif
				if (!thread_list->status) {
#ifdef PID_CHECK
					if (t->base_addr != thread_list->base_addr)
						LOG_INFO("thread base_addr has changed !!");
#endif
					/*  this is not a current thread  */
					thread_list->base_addr = t->base_addr;
					thread_list->status = 1;

					/*  we don 't update this field any more */

					/*thread_list->state = t->state;
					thread_list->oncpu = t->oncpu;
					thread_list->asid = t->asid;
					*/
					if (context)
						thread_list->context =
							cpu_context_read(target,
								thread_list->base_addr,
								&thread_list->thread_info_addr);
				} else {
					/*  it is a current thread no need to read context */
				}

				linux_os->thread_count++;
				found = 1;
				break;
			} else {
				found = 0;
				thread_list = thread_list->next;
			}
		}

		if (found == 0) {
			uint32_t base_addr;
			fill_task(target, t);
			get_name(target, t);
			retval = insert_into_threadlist(target, t);
			t->thread_info_addr = 0xdeadbeef;

			if (context)
				t->context =
					cpu_context_read(target, t->base_addr,
						&t->thread_info_addr);

			base_addr = next_task(target, t);
			t = calloc(1, sizeof(struct threads));
			t->base_addr = base_addr;
			linux_os->thread_count++;
		} else
			t->base_addr = next_task(target, t);
	}

	LOG_INFO("update thread done %" PRId64 ", mean%" PRId64 "\n",
		(timeval_ms() - start), (timeval_ms() - start) / loop);
	free(t);
	linux_os->threads_needs_update = 0;
	return ERROR_OK;
}

static int linux_gdb_thread_packet(struct target *target,
	struct connection *connection, char const *packet,
	int packet_size)
{
	int retval;
	struct linux_os *linux_os =
		(struct linux_os *)target->rtos->rtos_specific_params;

	if (linux_os->init_task_addr == 0xdeadbeef) {
		/* it has not been initialized */
		LOG_INFO("received thread request without init task address");
		gdb_put_packet(connection, "l", 1);
		return ERROR_OK;
	}

	retval = linux_get_tasks(target, 1);

	if (retval != ERROR_OK)
		return ERROR_TARGET_FAILURE;

	char *out_str = calloc(MAX_THREADS * 17 + 10, 1);
	char *tmp_str = out_str;
	tmp_str += sprintf(tmp_str, "m");
	struct threads *temp = linux_os->thread_list;

	while (temp != NULL) {
		tmp_str += sprintf(tmp_str, "%016" PRIx64, temp->threadid);
		temp = temp->next;
		if (temp)
			tmp_str += sprintf(tmp_str, ",");
	}

	gdb_put_packet(connection, out_str, strlen(out_str));
	free(out_str);
	return ERROR_OK;
}

static int linux_gdb_thread_update(struct target *target,
	struct connection *connection, char const *packet,
	int packet_size)
{
	int found = 0;
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	struct threads *temp = linux_os->thread_list;

	while (temp != NULL) {
		if (temp->threadid == linux_os->preupdtate_threadid_count + 1) {
			/*LOG_INFO("FOUND");*/
			found = 1;
			break;
		} else
			temp = temp->next;
	}

	if (found == 1) {
		/*LOG_INFO("INTO GDB THREAD UPDATE FOUNDING START TASK");*/
		char *out_strr = calloc(MAX_THREADS * 17 + 10, 1);
		char *tmp_strr = out_strr;
		tmp_strr += sprintf(tmp_strr, "m");
		/*LOG_INFO("CHAR MALLOC & M DONE");*/
		tmp_strr += sprintf(tmp_strr, "%016" PRIx64, temp->threadid);

		temp = temp->next;

		while (temp != NULL) {
			/*LOG_INFO("INTO GDB THREAD UPDATE WHILE");*/
			tmp_strr += sprintf(tmp_strr, ",");
			tmp_strr +=
				sprintf(tmp_strr, "%016" PRIx64, temp->threadid);
			temp = temp->next;
		}

		/*tmp_str[0] = 0;*/
		gdb_put_packet(connection, out_strr, strlen(out_strr));
		linux_os->preupdtate_threadid_count =
			linux_os->threadid_count - 1;
		free(out_strr);
	} else
		gdb_put_packet(connection, "l", 1);

	return ERROR_OK;
}

static int linux_thread_extra_info(struct target *target,
	struct connection *connection, char const *packet,
	int packet_size)
{
	int64_t threadid = 0;
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	sscanf(packet, "qThreadExtraInfo,%" SCNx64, &threadid);
	/*LOG_INFO("lookup extra info for thread %" SCNx64, threadid);*/
	struct threads *temp = linux_os->thread_list;

	while (temp != NULL) {
		if (temp->threadid == threadid) {
			char *pid = " PID: ";
			char *pid_current = "*PID: ";
			char *name = "Name: ";
			int str_size = strlen(pid) + strlen(name);
			char *tmp_str = calloc(1, str_size + 50);
			char *tmp_str_ptr = tmp_str;

			/*  discriminate current task */
			if (temp->status == 3)
				tmp_str_ptr += sprintf(tmp_str_ptr, "%s",
						pid_current);
			else
				tmp_str_ptr += sprintf(tmp_str_ptr, "%s", pid);

			tmp_str_ptr += sprintf(tmp_str_ptr, "%d, ", (int)temp->pid);
			sprintf(tmp_str_ptr, "%s", name);
			sprintf(tmp_str_ptr, "%s", temp->name);
			char *hex_str = calloc(1, strlen(tmp_str) * 2 + 1);
			size_t pkt_len = hexify(hex_str, (const uint8_t *)tmp_str,
				strlen(tmp_str), strlen(tmp_str) * 2 + 1);
			gdb_put_packet(connection, hex_str, pkt_len);
			free(hex_str);
			free(tmp_str);
			return ERROR_OK;
		}

		temp = temp->next;
	}

	LOG_INFO("thread not found");
	return ERROR_OK;
}

static int linux_gdb_T_packet(struct connection *connection,
	struct target *target, char const *packet, int packet_size)
{
	int64_t threadid;
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	int retval = ERROR_OK;
	sscanf(packet, "T%" SCNx64, &threadid);

	if (linux_os->threads_needs_update == 0) {
		struct threads *temp = linux_os->thread_list;
		struct threads *prev = NULL;

		while (temp != NULL) {
			if (temp->threadid == threadid) {
				if (temp->status != 0) {
					gdb_put_packet(connection, "OK", 2);
					return ERROR_OK;
				} else {
					/* delete item in the list   */
					linux_os->thread_list =
						liste_del_task(linux_os->thread_list,
							&temp, prev);
					linux_os->thread_count--;
					gdb_put_packet(connection, "E01", 3);
					return ERROR_OK;
				}
			}

			/*  for deletion  */
			prev = temp;
			temp = temp->next;
		}

		LOG_INFO("gdb requested status on non existing thread");
		gdb_put_packet(connection, "E01", 3);
		return ERROR_OK;

	} else {
		retval = linux_task_update(target, 1);
		struct threads *temp = linux_os->thread_list;

		while (temp != NULL) {
			if (temp->threadid == threadid) {
				if (temp->status == 1) {
					gdb_put_packet(connection, "OK", 2);
					return ERROR_OK;
				} else {
					gdb_put_packet(connection, "E01", 3);
					return ERROR_OK;
				}
			}

			temp = temp->next;
		}
	}

	return retval;
}

static int linux_gdb_h_packet(struct connection *connection,
	struct target *target, char const *packet, int packet_size)
{
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	struct current_thread *ct = linux_os->current_threads;

	/* select to display the current thread of the selected target */
	while ((ct != NULL) && (ct->core_id != target->coreid))
		ct = ct->next;

	int64_t current_gdb_thread_rq;

	if (linux_os->threads_lookup == 1) {
		if ((ct != NULL) && (ct->threadid == -1)) {
			ct = linux_os->current_threads;

			while ((ct != NULL) && (ct->threadid == -1))
				ct = ct->next;
		}

		if (ct == NULL) {
			/*  no current thread can be identified
			 *  any way with smp  */
			LOG_INFO("no current thread identified");
			/* attempt to display the name of the 2 threads identified with
			 * get_current */
			struct threads t;
			ct = linux_os->current_threads;

			while ((ct != NULL) && (ct->threadid == -1)) {
				t.base_addr = ct->TS;
				get_name(target, &t);
				LOG_INFO("name of unidentified thread %s",
					t.name);
				ct = ct->next;
			}

			gdb_put_packet(connection, "OK", 2);
			return ERROR_OK;
		}

		if (packet[1] == 'g') {
			sscanf(packet, "Hg%16" SCNx64, &current_gdb_thread_rq);

			if (current_gdb_thread_rq == 0) {
				target->rtos->current_threadid = ct->threadid;
				gdb_put_packet(connection, "OK", 2);
			} else {
				target->rtos->current_threadid =
					current_gdb_thread_rq;
				gdb_put_packet(connection, "OK", 2);
			}
		} else if (packet[1] == 'c') {
			sscanf(packet, "Hc%16" SCNx64, &current_gdb_thread_rq);

			if ((current_gdb_thread_rq == 0) ||
					(current_gdb_thread_rq == ct->threadid)) {
				target->rtos->current_threadid = ct->threadid;
				gdb_put_packet(connection, "OK", 2);
			} else
				gdb_put_packet(connection, "E01", 3);
		}
	} else
		gdb_put_packet(connection, "OK", 2);

	return ERROR_OK;
}

static int linux_thread_packet(struct connection *connection, char const *packet,
	int packet_size)
{
	int retval = ERROR_OK;
	struct current_thread *ct;
	struct target *target = get_target_from_connection(connection);
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;

	switch (packet[0]) {
		case 'T':		/* Is thread alive?*/

			linux_gdb_T_packet(connection, target, packet, packet_size);
			break;
		case 'H':		/* Set current thread */
			/*  ( 'c' for step and continue, 'g' for all other operations )*/
			/*LOG_INFO(" H packet received '%s'", packet);*/
			linux_gdb_h_packet(connection, target, packet, packet_size);
			break;
		case 'q':

			if (strncmp(packet, "qSymbol", 7) == 0) {
				if (rtos_qsymbol(connection, packet, packet_size) == 1) {
					linux_compute_virt2phys(target,
							target->rtos->symbols[INIT_TASK].address);
				}

				break;
			} else if (strncmp(packet, "qfThreadInfo", 12) == 0) {
				if (linux_os->thread_list == NULL) {
					retval = linux_gdb_thread_packet(target,
							connection,
							packet,
							packet_size);
					break;
				} else {
					retval = linux_gdb_thread_update(target,
							connection,
							packet,
							packet_size);
					break;
				}
			} else if (strncmp(packet, "qsThreadInfo", 12) == 0) {
				gdb_put_packet(connection, "l", 1);
				break;
			} else if (strncmp(packet, "qThreadExtraInfo,", 17) == 0) {
				linux_thread_extra_info(target, connection, packet,
						packet_size);
				break;
			} else {
				retval = GDB_THREAD_PACKET_NOT_CONSUMED;
				break;
			}

		case 'Q':
			/* previously response was : thread not found
			 * gdb_put_packet(connection, "E01", 3); */
			retval = GDB_THREAD_PACKET_NOT_CONSUMED;
			break;
		case 'c':
		case 's': {
			if (linux_os->threads_lookup == 1) {
				ct = linux_os->current_threads;

				while ((ct != NULL) && (ct->core_id) != target->coreid)
					ct = ct->next;

				if ((ct != NULL) && (ct->threadid == -1)) {
					ct = linux_os->current_threads;

					while ((ct != NULL) && (ct->threadid == -1))
						ct = ct->next;
				}

				if ((ct != NULL) && (ct->threadid !=
						 target->rtos->current_threadid)
				&& (target->rtos->current_threadid != -1))
					LOG_WARNING("WARNING! current GDB thread do not match "
							"current thread running. "
							"Switch thread in GDB to threadid %d",
							(int)ct->threadid);

				LOG_INFO("threads_needs_update = 1");
				linux_os->threads_needs_update = 1;
			}
		}

		/* if a packet handler returned an error, exit input loop */
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int linux_os_smp_init(struct target *target)
{
	struct target_list *head;
	/* keep only target->rtos */
	struct rtos *rtos = target->rtos;
	struct linux_os *os_linux =
		(struct linux_os *)rtos->rtos_specific_params;
	struct current_thread *ct;
	head = target->head;

	while (head != (struct target_list *)NULL) {
		if (head->target->rtos != rtos) {
			struct linux_os *smp_os_linux =
				(struct linux_os *)head->target->rtos->rtos_specific_params;
			/*  remap smp target on rtos  */
			free(head->target->rtos);
			head->target->rtos = rtos;
			/*  reuse allocated ct */
			ct = smp_os_linux->current_threads;
			ct->threadid = -1;
			ct->TS = 0xdeadbeef;
			ct->core_id = head->target->coreid;
			os_linux->current_threads =
				add_current_thread(os_linux->current_threads, ct);
			os_linux->nr_cpus++;
			free(smp_os_linux);
		}

		head = head->next;
	}

	return ERROR_OK;
}

static int linux_os_create(struct target *target)
{
	struct linux_os *os_linux = calloc(1, sizeof(struct linux_os));
	struct current_thread *ct = calloc(1, sizeof(struct current_thread));
	LOG_INFO("linux os creation\n");
	os_linux->init_task_addr = 0xdeadbeef;
	os_linux->name = "linux";
	os_linux->thread_list = NULL;
	os_linux->thread_count = 0;
	target->rtos->current_threadid = -1;
	os_linux->nr_cpus = 1;
	os_linux->threads_lookup = 0;
	os_linux->threads_needs_update = 0;
	os_linux->threadid_count = 1;
	os_linux->current_threads = NULL;
	target->rtos->rtos_specific_params = os_linux;
	ct->core_id = target->coreid;
	ct->threadid = -1;
	ct->TS = 0xdeadbeef;
	os_linux->current_threads =
		add_current_thread(os_linux->current_threads, ct);
	/*  overload rtos thread default handler */
	target->rtos->gdb_thread_packet = linux_thread_packet;
	/*  initialize a default virt 2 phys translation */
	os_linux->phys_mask = ~0xc0000000;
	os_linux->phys_base = 0x0;
	return JIM_OK;
}

static char *linux_ps_command(struct target *target)
{
	struct linux_os *linux_os = (struct linux_os *)
		target->rtos->rtos_specific_params;
	int retval = ERROR_OK;
	char *display;

	if (linux_os->threads_lookup == 0)
		retval = linux_get_tasks(target, 1);
	else {
		if (linux_os->threads_needs_update != 0)
			retval = linux_task_update(target, 0);
	}

	if (retval == ERROR_OK) {
		struct threads *temp = linux_os->thread_list;
		char *tmp;
		LOG_INFO("allocation for %d threads line",
			linux_os->thread_count);
		display = calloc((linux_os->thread_count + 2) * 80, 1);

		if (!display)
			goto error;

		tmp = display;
		tmp += sprintf(tmp, "PID\t\tCPU\t\tASID\t\tNAME\n");
		tmp += sprintf(tmp, "---\t\t---\t\t----\t\t----\n");

		while (temp != NULL) {
			if (temp->status) {
				if (temp->context)
					tmp +=
						sprintf(tmp,
							"%" PRIu32 "\t\t%" PRIu32 "\t\t%" PRIx32 "\t\t%s\n",
							temp->pid, temp->oncpu,
							temp->asid, temp->name);
				else
					tmp +=
						sprintf(tmp,
							"%" PRIu32 "\t\t%" PRIu32 "\t\t%" PRIx32 "\t\t%s\n",
							temp->pid, temp->oncpu,
							temp->asid, temp->name);
			}

			temp = temp->next;
		}

		return display;
	}

error:
	display = calloc(40, 1);
	sprintf(display, "linux_ps_command failed\n");
	return display;
}
