/***************************************************************************
 *   Copyright (C) 2017 by Intel Corporation                               *
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

#include "helper/log.h"
#include "helper/types.h"
#include "rtos.h"
#include "rtos_standard_stackings.h"
#include "target/target.h"
#include "target/target_type.h"
#include "target/armv7m.h"
#include "target/arc.h"

#define UNIMPLEMENTED 0xFFFFFFFFU

/*ARC specific defines*/
#define ARC_AUX_SEC_BUILD_REG 0xdb
#define ARC_REG_NUM 38

struct zephyr_thread {
	uint32_t ptr, next_ptr;
	uint32_t entry;
	uint32_t stack_pointer;
	uint8_t state;
	uint8_t user_options;
	int8_t prio;
	char name[64];
};

enum zephyr_offsets {
	OFFSET_VERSION,
	OFFSET_K_CURR_THREAD,
	OFFSET_K_THREADS,
	OFFSET_T_ENTRY,
	OFFSET_T_NEXT_THREAD,
	OFFSET_T_STATE,
	OFFSET_T_USER_OPTIONS,
	OFFSET_T_PRIO,
	OFFSET_T_STACK_POINTER,
	OFFSET_T_NAME,
	OFFSET_T_ARCH,
	OFFSET_T_PREEMPT_FLOAT,
	OFFSET_T_COOP_FLOAT,
	OFFSET_MAX
};

struct zephyr_params {
	const char *target_name;
	uint8_t size_width;
	uint8_t pointer_width;
	uint32_t num_offsets;
	uint32_t offsets[OFFSET_MAX];
	const struct rtos_register_stacking *callee_saved_stacking;
	const struct rtos_register_stacking *cpu_saved_nofp_stacking;
	const struct rtos_register_stacking *cpu_saved_fp_stacking;
};

static const struct stack_register_offset arm_callee_saved[] = {
	{ ARMV7M_R13, 32, 32 },
	{ 4,  0,  32 },
	{ 5,  4,  32 },
	{ 6,  8,  32 },
	{ 7,  12, 32 },
	{ 8,  16, 32 },
	{ 9,  20, 32 },
	{ 10, 24, 32 },
	{ 11, 28, 32 },
};

static const struct stack_register_offset arc_callee_saved[] = {
	{ 13,  0,  32 },
	{ 14,  4,  32 },
	{ 15,  8,  32 },
	{ 16,  12,  32 },
	{ 17,  16,  32 },
	{ 18,  20,  32 },
	{ 19,  24,  32 },
	{ 20,  28,  32 },
	{ 21,  32,  32 },
	{ 22,  36,  32 },
	{ 23,  40,  32 },
	{ 24,  44,  32 },
	{ 25,  48,  32 },
	{ 26,  52,  32 },
	{ 27,  56,  32 },
	{ 30,  60,  32 }
};
static const struct rtos_register_stacking arm_callee_saved_stacking = {
	.stack_registers_size = 36,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arm_callee_saved),
	.register_offsets = arm_callee_saved,
};

static const struct rtos_register_stacking arc_callee_saved_stacking = {
	.stack_registers_size = sizeof(arc_callee_saved),
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arc_callee_saved),
	.register_offsets = arc_callee_saved,
};

static const struct stack_register_offset arm_cpu_saved[] = {
	{ ARMV7M_R0,   0,  32 },
	{ ARMV7M_R1,   4,  32 },
	{ ARMV7M_R2,   8,  32 },
	{ ARMV7M_R3,   12, 32 },
	{ ARMV7M_R4,   -1, 32 },
	{ ARMV7M_R5,   -1, 32 },
	{ ARMV7M_R6,   -1, 32 },
	{ ARMV7M_R7,   -1, 32 },
	{ ARMV7M_R8,   -1, 32 },
	{ ARMV7M_R9,   -1, 32 },
	{ ARMV7M_R10,  -1, 32 },
	{ ARMV7M_R11,  -1, 32 },
	{ ARMV7M_R12,  16, 32 },
	{ ARMV7M_R13,  -2, 32 },
	{ ARMV7M_R14,  20, 32 },
	{ ARMV7M_PC,   24, 32 },
	{ ARMV7M_xPSR, 28, 32 },
};

static struct stack_register_offset arc_cpu_saved[] = {
	{ 0,   -1,  32 },
	{ 1,   -1,  32 },
	{ 2,   -1,  32 },
	{ 3,   -1,  32 },
	{ 4,   -1,  32 },
	{ 5,   -1,  32 },
	{ 6,   -1,  32 },
	{ 7,   -1,  32 },
	{ 8,   -1,  32 },
	{ 9,   -1,  32 },
	{ 10,   -1,  32 },
	{ 11,   -1,  32 },
	{ 12,   -1,  32 },
	{ 13,   -1,  32 },
	{ 14,   -1,  32 },
	{ 15,   -1,  32 },
	{ 16,   -1,  32 },
	{ 17,   -1,  32 },
	{ 18,   -1,  32 },
	{ 19,   -1,  32 },
	{ 20,   -1,  32 },
	{ 21,   -1,  32 },
	{ 22,   -1,  32 },
	{ 23,   -1,  32 },
	{ 24,   -1,  32 },
	{ 25,   -1,  32 },
	{ 26,   -1,  32 },
	{ 27,   -1,  32 },
	{ 28,   -1,  32 },
	{ 29,   -1,  32 },
	{ 30,   -1,  32 },
	{ 31,   0,  32 },
	{ 60,   -1, 32 },
	{ 63,   -1, 32 },
	{ 64,   -1,  32 },
	{ 65,   -1,  32 },
	{ 66,   -1,  32 },
	{ 67,   4,  32 }
};

static int64_t zephyr_Cortex_M_stack_align(struct target *target,
								const uint8_t *stack_data,
								const struct rtos_register_stacking *stacking,
								int64_t stack_ptr)
{
	const int XPSR_OFFSET = 28;
	return rtos_Cortex_M_stack_align(target, stack_data, stacking, stack_ptr,
									 XPSR_OFFSET);
}

static const struct rtos_register_stacking arm_cpu_saved_nofp_stacking = {
	.stack_registers_size = 32,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arm_cpu_saved),
	.calculate_process_stack = zephyr_Cortex_M_stack_align,
	.register_offsets = arm_cpu_saved,
};

static const struct rtos_register_stacking arm_cpu_saved_fp_stacking = {
	.stack_registers_size = 32 + 18 * 4,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arm_cpu_saved),
	.calculate_process_stack = zephyr_Cortex_M_stack_align,
	.register_offsets = arm_cpu_saved,
};

/* stack_registers_size is 8 because besides caller registers
 * there are only blink and Status32 registers on stack left */
static struct rtos_register_stacking arc_cpu_saved_stacking = {
	.stack_registers_size = 8,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arc_cpu_saved),
	.register_offsets = arc_cpu_saved,
};


static struct zephyr_params zephyr_params_list[] = {
	{
		.target_name = "cortex_m",
		.pointer_width = 4,
		.callee_saved_stacking = &arm_callee_saved_stacking,
		.cpu_saved_nofp_stacking = &arm_cpu_saved_nofp_stacking,
		.cpu_saved_fp_stacking = &arm_cpu_saved_fp_stacking,
	},
	{
		.target_name = "arcv2",
		.pointer_width = 4,
		.callee_saved_stacking = &arc_callee_saved_stacking,
		.cpu_saved_nofp_stacking = &arc_cpu_saved_stacking,
	},
	{
		.target_name = NULL
	}
};

static bool zephyr_detect_rtos(struct target *target);
static int zephyr_create(struct target *target);
static int zephyr_update_threads(struct rtos *rtos);
static int zephyr_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int zephyr_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);

struct rtos_type zephyr_rtos = {
	.name = "Zephyr",

	.detect_rtos = zephyr_detect_rtos,
	.create = zephyr_create,
	.update_threads = zephyr_update_threads,
	.get_thread_reg_list = zephyr_get_thread_reg_list,
	.get_symbol_list_to_lookup = zephyr_get_symbol_list_to_lookup,
};

enum zephyr_symbol_values {
	ZEPHYR_VAL__KERNEL,
	ZEPHYR_VAL__KERNEL_OPENOCD_OFFSETS,
	ZEPHYR_VAL__KERNEL_OPENOCD_SIZE_T_SIZE,
	ZEPHYR_VAL__KERNEL_OPENOCD_NUM_OFFSETS,
	ZEPHYR_VAL_COUNT
};

static const symbol_table_elem_t zephyr_symbol_list[] = {
	{ .symbol_name = "_kernel", .optional = false },
	{ .symbol_name = "_kernel_openocd_offsets", .optional = false },
	{ .symbol_name = "_kernel_openocd_size_t_size", .optional = false },
	{ .symbol_name = "_kernel_openocd_num_offsets", .optional = true },
	{ .symbol_name = NULL }
};

struct array {
	void *ptr;
	size_t elements;
};

static void array_init(struct array *array)
{
	array->ptr = NULL;
	array->elements = 0;
}

static void array_free(struct array *array)
{
	free(array->ptr);
	array_init(array);
}

static void *array_append(struct array *array, size_t size)
{
	if (!(array->elements % 16)) {
		void *ptr = realloc(array->ptr, (array->elements + 16) * size);

		if (!ptr)
			return NULL;

		array->ptr = ptr;
	}

	return (unsigned char *)array->ptr + (array->elements++) * size;
}

static void *array_detach_ptr(struct array *array)
{
	void *ptr = array->ptr;

	array_init(array);

	return ptr;
}

static uint32_t zephyr_kptr(const struct rtos *rtos, enum zephyr_offsets off)
{
	const struct zephyr_params *params = rtos->rtos_specific_params;

	return rtos->symbols[ZEPHYR_VAL__KERNEL].address + params->offsets[off];
}

static int zephyr_fetch_thread(const struct rtos *rtos,
							   struct zephyr_thread *thread, uint32_t ptr)
{
	const struct zephyr_params *param = rtos->rtos_specific_params;
	int retval;

	thread->ptr = ptr;

	retval = target_read_u32(rtos->target, ptr + param->offsets[OFFSET_T_ENTRY],
							 &thread->entry);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(rtos->target,
							 ptr + param->offsets[OFFSET_T_NEXT_THREAD],
							 &thread->next_ptr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(rtos->target,
							 ptr + param->offsets[OFFSET_T_STACK_POINTER],
							 &thread->stack_pointer);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u8(rtos->target, ptr + param->offsets[OFFSET_T_STATE],
							&thread->state);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u8(rtos->target,
							ptr + param->offsets[OFFSET_T_USER_OPTIONS],
							&thread->user_options);
	if (retval != ERROR_OK)
		return retval;

	uint8_t prio;
	retval = target_read_u8(rtos->target,
							ptr + param->offsets[OFFSET_T_PRIO], &prio);
	if (retval != ERROR_OK)
		return retval;
	thread->prio = (int8_t)prio;

	thread->name[0] = '\0';
	if (param->offsets[OFFSET_T_NAME] != UNIMPLEMENTED) {

		retval = target_read_buffer(rtos->target,  ptr + param->offsets[OFFSET_T_NAME],
							sizeof(thread->name) - 1, (uint8_t *)thread->name);
		if (retval != ERROR_OK)
			return retval;

		thread->name[sizeof(thread->name) - 1] = '\0';
	}

	LOG_DEBUG("Fetched thread%" PRIx32 ": {entry@0x%" PRIx32
		", state=%" PRIu8 ", useropts=%" PRIu8 ", prio=%" PRId8 "}",
		ptr, thread->entry, thread->state, thread->user_options, thread->prio);

	return ERROR_OK;
}

static int zephyr_fetch_thread_list(struct rtos *rtos, uint32_t current_thread)
{
	struct array thread_array;
	struct zephyr_thread thread;
	struct thread_detail *td;
	int64_t curr_id;
	uint32_t curr;
	int retval;

	retval = target_read_u32(rtos->target, zephyr_kptr(rtos, OFFSET_K_THREADS),
		&curr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not fetch current thread pointer");
		return retval;
	}

	array_init(&thread_array);

	for (curr_id = -1; curr; curr = thread.next_ptr) {
		retval = zephyr_fetch_thread(rtos, &thread, curr);
		if (retval < 0)
			goto error;

		td = array_append(&thread_array, sizeof(*td));
		if (!td)
			goto error;

		td->threadid = thread.ptr;
		td->exists = true;

		if (thread.name[0])
			td->thread_name_str = strdup(thread.name);
		else
			td->thread_name_str = alloc_printf("thr_%" PRIx32 "_%" PRIx32,
											   thread.entry, thread.ptr);
		td->extra_info_str = alloc_printf("prio:%" PRId8 ",useropts:%" PRIu8,
										  thread.prio, thread.user_options);
		if (!td->thread_name_str || !td->extra_info_str)
			goto error;

		if (td->threadid == current_thread)
			curr_id = (int64_t)thread_array.elements - 1;
	}

	LOG_DEBUG("Got information for %zu threads", thread_array.elements);

	rtos_free_threadlist(rtos);

	rtos->thread_count = (int)thread_array.elements;
	rtos->thread_details = array_detach_ptr(&thread_array);

	rtos->current_threadid = curr_id;
	rtos->current_thread = current_thread;

	return ERROR_OK;

error:
	td = thread_array.ptr;
	for (size_t i = 0; i < thread_array.elements; i++) {
		free(td[i].thread_name_str);
		free(td[i].extra_info_str);
	}

	array_free(&thread_array);

	return ERROR_FAIL;
}

static int zephyr_update_threads(struct rtos *rtos)
{
	struct zephyr_params *param;
	int retval;

	if (rtos->rtos_specific_params == NULL)
		return ERROR_FAIL;

	param = (struct zephyr_params *)rtos->rtos_specific_params;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for Zephyr");
		return ERROR_FAIL;
	}

	if (rtos->symbols[ZEPHYR_VAL__KERNEL].address == 0) {
		LOG_ERROR("Can't obtain kernel struct from Zephyr");
		return ERROR_FAIL;
	}
	if (rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_OFFSETS].address == 0) {
		LOG_ERROR("Please build Zephyr with CONFIG_OPENOCD option set");
		return ERROR_FAIL;
	}

	retval = target_read_u8(rtos->target,
		rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_SIZE_T_SIZE].address,
		&param->size_width);
	if (retval != ERROR_OK) {
		LOG_ERROR("Couldn't determine size of size_t from host");
		return retval;
	}
	if (param->size_width != 4) {
		LOG_ERROR("Only size_t of 4 bytes are supported");
		return ERROR_FAIL;
	}

	if (rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_NUM_OFFSETS].address) {
		retval = target_read_u32(rtos->target,
				rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_NUM_OFFSETS].address,
				&param->num_offsets);
		if (retval != ERROR_OK) {
			LOG_ERROR("Couldn't not fetch number of offsets from Zephyr");
			return retval;
		}
		if (param->num_offsets <= OFFSET_T_STACK_POINTER) {
			LOG_ERROR("Number of offsets too small");
			return ERROR_FAIL;
		}
	} else {
		param->num_offsets = OFFSET_VERSION + 1;
	}
	/* We can fetch the whole array for version 0, as they're supposed
	 * to grow only */
	uint32_t address;
	address  = rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_OFFSETS].address;
	for (size_t i = 0; i < OFFSET_MAX; i++, address += param->size_width) {
		if (i >= param->num_offsets) {
			param->offsets[i] = UNIMPLEMENTED;
			continue;
		}
		retval = target_read_u32(rtos->target, address, &param->offsets[i]);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not fetch offsets from Zephyr");
			return ERROR_FAIL;
		}
		if (param->num_offsets == OFFSET_VERSION + 1 && i == OFFSET_VERSION) {
			switch (param->offsets[OFFSET_VERSION]) {
			case 0:
				param->num_offsets = OFFSET_T_STACK_POINTER + 1;
				break;
			case 1:
				param->num_offsets = OFFSET_T_COOP_FLOAT + 1;
				break;
			}
		}
	}
	if (param->offsets[OFFSET_VERSION] > 1) {
		LOG_ERROR("Unexpected OpenOCD support version %" PRIu32,
				  param->offsets[OFFSET_VERSION]);
		return ERROR_FAIL;
	}

	LOG_DEBUG("Zephyr OpenOCD support version %" PRId32,
			  param->offsets[OFFSET_VERSION]);

	uint32_t current_thread;
	retval = target_read_u32(rtos->target,
		zephyr_kptr(rtos, OFFSET_K_CURR_THREAD), &current_thread);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not obtain current thread ID");
		return retval;
	}

	retval = zephyr_fetch_thread_list(rtos, current_thread);
	if (retval < 0) {
		LOG_ERROR("Could not obtain thread list");
		return retval;
	}

	return ERROR_OK;
}

static int zephyr_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	struct zephyr_params *params;
	const struct rtos_register_stacking *stacking;
	struct rtos_reg *callee_saved_reg_list;
	int num_callee_saved_regs;
	int64_t addr;
	int retval;

	LOG_INFO("Getting thread %" PRId64 " reg list", thread_id);

	if (rtos == NULL)
		return ERROR_FAIL;

	if (thread_id == 0)
		return ERROR_FAIL;

	params = rtos->rtos_specific_params;
	if (params == NULL)
		return ERROR_FAIL;

	addr = thread_id + params->offsets[OFFSET_T_STACK_POINTER]
		 - params->callee_saved_stacking->register_offsets[0].offset;

	/* ARCv2 specific implementation */
	if (!strcmp(params->target_name, "arcv2")) {
		uint32_t real_stack_addr;

		/* Getting real stack addres from Kernel thread struct */
		retval = target_read_u32(rtos->target, addr, &real_stack_addr);
		if (retval != ERROR_OK)
			return retval;

		/* Getting callee registers */
		retval = rtos_generic_stack_read(rtos->target,
				params->callee_saved_stacking,
				real_stack_addr, &callee_saved_reg_list,
				&num_callee_saved_regs);
		if (retval < 0)
			return retval;

		stacking = params->cpu_saved_nofp_stacking;

		/* Getting blink and status32 registers */
		retval = rtos_generic_stack_read(rtos->target, stacking,
				real_stack_addr + num_callee_saved_regs * 4,
				reg_list, num_regs);
		if (retval >= 0)
			for (int i = 0; i < num_callee_saved_regs; i++)
				buf_cpy(callee_saved_reg_list[i].value,
					(*reg_list)[callee_saved_reg_list[i].number].value,
					callee_saved_reg_list[i].size);

		/* The blink, sp, pc offsets in arc_cpu_saved structure may be changed,
		 * but the registers number shall not. So the next code searches the
		 * offsetst of these registers in arc_cpu_saved structure. */
		unsigned short blink_offset = 0, pc_offset = 0, sp_offset = 0;
		for (uint32_t i = 0; i < sizeof(arc_cpu_saved) / sizeof(struct stack_register_offset); i++) {
			if (arc_cpu_saved[i].number == CORE_BLINK_NUM)
				blink_offset = i;
			if (arc_cpu_saved[i].number == CORE_SP_NUM)
				sp_offset = i;
			if (arc_cpu_saved[i].number == CORE_PC_NUM)
				pc_offset = i;
		}

		if (blink_offset == 0 || sp_offset == 0 || pc_offset == 0) {
			LOG_DEBUG("Basic registers offsets are missing, check <arc_cpu_saved> struct");
			return ERROR_FAIL;
		}

		/* Put blink value into PC */
		buf_cpy((*reg_list)[blink_offset].value, (*reg_list)[pc_offset].value, sizeof(uint32_t) * 8);

		/* Put address after callee/caller in SP.
		 * Here stack_p is similar to callee_saved_reg_list[i].value.
		 * Each callee_saved_reg_list[i].value is 8-size array of uint8_t, see rtos.h */
		uint8_t stack_p[8];
		int64_t stack_top;

		stack_top = real_stack_addr + num_callee_saved_regs * 4 + arc_cpu_saved_stacking.stack_registers_size;
		memcpy(stack_p, &(stack_top), 8);
		buf_cpy(stack_p, (*reg_list)[sp_offset].value, sizeof(uint32_t) * 8);
	}
	/* ARM Cortex-M-specific implementation */
	else if (!strcmp(params->target_name, "cortex_m")) {
		retval = rtos_generic_stack_read(rtos->target,
				params->callee_saved_stacking,
				addr, &callee_saved_reg_list,
				&num_callee_saved_regs);
		if (retval < 0)
			return retval;

		addr = target_buffer_get_u32(rtos->target,
						callee_saved_reg_list[0].value);

		if (params->offsets[OFFSET_T_PREEMPT_FLOAT] != UNIMPLEMENTED)
			stacking = params->cpu_saved_fp_stacking;
		else
			stacking = params->cpu_saved_nofp_stacking;

		retval = rtos_generic_stack_read(rtos->target, stacking, addr, reg_list,
										num_regs);
		if (retval >= 0)
			for (int i = 1; i < num_callee_saved_regs; i++)
				buf_cpy(callee_saved_reg_list[i].value,
					(*reg_list)[callee_saved_reg_list[i].number].value,
					callee_saved_reg_list[i].size);
	}
	/* Unsupported target target */
	else {
		LOG_DEBUG("Error: Unsupported target: should be coretx_m or arcv2.");
		return ERROR_FAIL;
	}
	free(callee_saved_reg_list);

	return retval;
}

static int zephyr_get_symbol_list_to_lookup(symbol_table_elem_t **symbol_list)
{
	*symbol_list = malloc(sizeof(zephyr_symbol_list));
	if (!*symbol_list)
		return ERROR_FAIL;

	memcpy(*symbol_list, zephyr_symbol_list, sizeof(zephyr_symbol_list));
	return ERROR_OK;
}

static bool zephyr_detect_rtos(struct target *target)
{
	if (target->rtos->symbols == NULL) {
		LOG_INFO("Zephyr: no symbols while detecting RTOS");
		return false;
	}

	for (enum zephyr_symbol_values symbol = ZEPHYR_VAL__KERNEL;
					symbol != ZEPHYR_VAL_COUNT; symbol++) {
		LOG_INFO("Zephyr: does it have symbol %d (%s)?", symbol,
			target->rtos->symbols[symbol].optional ? "optional" : "mandatory");

		if (target->rtos->symbols[symbol].optional)
			continue;
		if (target->rtos->symbols[symbol].address == 0)
			return false;
	}

	LOG_INFO("Zephyr: all mandatory symbols found");

	return true;
}

static int zephyr_create(struct target *target)
{
	;
	const char *name;

	name = target_type_name(target);
	if (!strcmp(name, "hla_target")) {
		name = target_get_gdb_arch(target);
		if (!name) {
			LOG_ERROR("Zephyr: failed to determine target type");
			return ERROR_FAIL;
		}
	}

	LOG_INFO("Zephyr: looking for target: %s", name);

	/* ARC specific, check if EM target has security subsystem
	 * In case of ARC_HAS_SECURE zephyr option enabled
	 * the thread stack contains blink,sec_stat,status32 register
	 * values. If ARC_HAS_SECURE is disabled, only blink and status32
	 * register values are saved on stack. */
	if (!strcmp(name, "arcv2")) {
		uint32_t value;
		struct arc_common *arc = target_to_arc(target);
		/* Reading SEC_BUILD bcr */
		CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, ARC_AUX_SEC_BUILD_REG, &value));
		if (value != 0) {
			LOG_DEBUG("ARC EM board has security subsystem, changing offsets");
			arc_cpu_saved[ARC_REG_NUM-1].offset = 8;
			/* After reading callee registers in stack
			 * now blink,sec_stat,status32 registers
			 * are located. */
			arc_cpu_saved_stacking.stack_registers_size = 12;
		}
	}

	if (!strcmp(name, "arm"))
		name = "cortex_m";

	for (struct zephyr_params *p = zephyr_params_list; p->target_name; p++) {
		if (!strcmp(p->target_name, name)) {
			LOG_INFO("Zephyr: target known, params at %p", p);
			target->rtos->rtos_specific_params = p;
			return ERROR_OK;
		}
	}

	LOG_ERROR("Could not find target in Zephyr compatibility list");
	return ERROR_FAIL;
}
