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
#include "target/armv7m.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "rtos_standard_stackings.h"
#include "rtos_ecos_stackings.h"
#include "server/gdb_server.h"

/* INVESTIGATE: Unfortunately for the moment we are limited to returning the
 * hardwired register count (ARMV7M_NUM_CORE_REGS for Cortex-M) since if (for
 * Cortex-M) we append BASEPRI or the VFP information to the returned register
 * list we get a "Remote 'g' packet reply is too long (expected 68 bytes, got
 * XXX bytes)" error from GDB. We need to ascertain the best solution to
 * returning per-thread BASEPRI and VFP information to GDB.
 *
 * So, for the moment, we continue to use the hardwired limit for the depth of
 * the returned register description vector. The rtos_standard_stackings.c also
 * just seem to provide the main core regs for the Cortex_M* targets regardless
 * of whether FPU is present/enabled.
 *
 * However, this code is written with the expectation that we may eventually be
 * able to provide more register information ("m-system" and "vfp" for example)
 * and also with the expectation of supporting different register sets being
 * returned depending on the per-thread Cortex-M eCos contex_m for
 * example. Hence the fact that the eCos_stack_layout_*() functions below allow
 * for the stack context descriptor vector to be returned by those calls
 * allowing for eventual support where this code will potentially cache
 * different sets of register descriptors for the different shapes of contexts
 * in a *single* application/binary run-time. In preperation for that work the
 * _EXTENDED_REGLIST manifest controls some extra code as work-in-progress for
 * that support. */

/* #define _EXTENDED_REGLIST (1) */

struct eCos_params;

static bool eCos_detect_rtos(struct target *target);
static int eCos_create(struct target *target);
static int eCos_update_threads(struct rtos *rtos);
static int eCos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id, struct rtos_reg **reg_list, int *num_regs);
static int eCos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[]);
static int eCos_stack_layout_cortexm(struct rtos *rtos, struct eCos_params *param,
				int64_t stack_ptr, const struct rtos_register_stacking **si);
static int eCos_stack_layout_arm(struct rtos *rtos, struct eCos_params *param,
				int64_t stack_ptr, const struct rtos_register_stacking **si);

/* The current eCos thread IDentifier is a 12-bit value with 0 being an unused
 * value. */
#define ECOS_MAX_THREAD_COUNT (4095)

struct eCos_thread_state {
	int value;
	const char *desc;
};

/* The status is actually a logical-OR bitmask of states: */
typedef enum __attribute__ ((__packed__)) {
	RUNNING    = 0,
	SLEEPING   = 1,
	COUNTSLEEP = 2,
	SUSPENDED  = 4,
	CREATING   = 8,
	EXITED     = 16,
	SLEEPSET   = (SLEEPING | COUNTSLEEP)
} eCos_thread_state_flags;

/* Cyg_Thread:: reason codes for wake and sleep fields: */
static const struct eCos_thread_state ecos_thread_reasons[] = {
	{ 0, "NONE" }, /* normally indicates "not yet started" */
	{ 1, "WAIT" }, /* wait with no timeout */
	{ 2, "DELAY" }, /* simple time delay */
	{ 3, "TIMEOUT" }, /* wait with timeout *or* timeout expired */
	{ 4, "BREAK" }, /* forced break out of sleep */
	{ 5, "DESTRUCT" }, /* wait on object being destroyed */
	{ 6, "EXIT" }, /* forced termination */
	{ 7, "DONE" } /* wait/delay completed */
};

static const char * const target_cortex_m[] = {
	"cortex_m",
	"hla_target",
	NULL };

static const char * const target_arm[] = {
	"cortex_a",
	"arm7tdmi",
	"arm720t",
	"arm9tdmi",
	"arm920t",
	"arm926ejs",
	"arm946e",
	"arm966e",
	"arm11",
	NULL };

/* Since individual eCos application configurations may have different thread
 * object structure layouts depending on the actual build-time enabled features
 * we provide support for applications built containing the relevant symbolic
 * support to match the actual application binary being debugged, rather than
 * relying on a set of default/fixed (and potentially incorrect)
 * offsets. However, for backwards compatibility, we do *NOT* enforce the
 * requirement for the common extra helper symbols to be present to allow the
 * fallback to the simple fixed CM3 model to avoid affecting existing users of
 * older eCos worlds. Similarly we need to provide support for per-thread
 * register context offsets, as well as for per-application-configurations,
 * since some targets can have different stacked state on a per-thread basis
 * (e.g. "cortex_m"). This is why the stacking_info is now set at run-time
 * rather than being fixed. */

struct eCos_params {
	const char * const *target_names; /* NULL terminated list of targets */
	int (*target_stack_layout)(struct rtos *rtos, struct eCos_params *param,
		int64_t stack_ptr, const struct rtos_register_stacking **si);
	bool flush_common;
	unsigned char pointer_width;
	unsigned char uid_width;
	unsigned char state_width;
	unsigned char thread_stack_offset;
	unsigned char thread_name_offset;
	unsigned char thread_state_offset;
	unsigned char thread_next_offset;
	unsigned char thread_uniqueid_offset;
	const struct rtos_register_stacking *stacking_info;
#if defined(_EXTENDED_REGLIST)
	const struct rtos_register_stacking *stacking_info_fpu;
#endif /* _EXTENDED_REGLIST */
};

/* As mentioned above we provide default offset values for the "cortex_m"
 * targets for backwards compatibility with older eCos application builds and
 * previous users of this RTOS specific support that do not have the
 * configuration specific offsets provided in the symbol table. The support for
 * other targets (e.g. "cortex_a") we do expect the application to provide the
 * required symbolic information. We do not populate the stacking_info reference
 * until we have had a chance to interrogate the symbol table. */

static struct eCos_params eCos_params_list[] = {
	{
	.target_names = target_cortex_m,
	.pointer_width = 4,
	.uid_width = 2,
	.state_width = 4,
	.thread_stack_offset = 0x0c,
	.thread_name_offset = 0x9c,
	.thread_state_offset = 0x3c,
	.thread_next_offset = 0xa0,
	.thread_uniqueid_offset = 0x4c,
	.target_stack_layout = eCos_stack_layout_cortexm,
#if defined(_EXTENDED_REGLIST)
	.stacking_info_fpu = NULL,
#endif /* _EXTENDED_REGLIST */
	.stacking_info = NULL
	},
	{
	.target_names = target_arm,
	.pointer_width = 0,
	.uid_width = 0,
	.state_width = 0,
	.thread_stack_offset = 0,
	.thread_name_offset = 0,
	.thread_state_offset = 0,
	.thread_next_offset = 0,
	.thread_uniqueid_offset = 0,
	.target_stack_layout = eCos_stack_layout_arm,
#if defined(_EXTENDED_REGLIST)
	.stacking_info_fpu = NULL,
#endif /* _EXTENDED_REGLIST */
	.stacking_info = NULL
	}
};

#define ECOS_NUM_PARAMS ARRAY_SIZE(eCos_params_list)

/* To eventually allow for more than just the ARMV7M_NUM_CORE_REGS to be
 * returned by the Cortex-M support, and to avoid run-time lookups we manually
 * maintain our own mapping for the supplied stack register vector entries. This
 * enum needs to match the rtos_ecos_regoff_cortexm[] vector. Admittedly the
 * initial indices just match the corresponding ARMV7M_R* definitions, but after
 * the base registers the ARMV7M_* number space does not match the vector we
 * wish to populate in this eCos support code. */
enum ecos_reglist_cortexm {
	ECOS_REGLIST_R0 = 0,
	ECOS_REGLIST_R1,
	ECOS_REGLIST_R2,
	ECOS_REGLIST_R3,
	ECOS_REGLIST_R4,
	ECOS_REGLIST_R5,
	ECOS_REGLIST_R6,
	ECOS_REGLIST_R7,
	ECOS_REGLIST_R8,
	ECOS_REGLIST_R9,
	ECOS_REGLIST_R10,
	ECOS_REGLIST_R11,
	ECOS_REGLIST_R12,
	ECOS_REGLIST_R13,
	ECOS_REGLIST_R14,
	ECOS_REGLIST_PC,
	ECOS_REGLIST_xPSR,	/* ARMV7M_NUM_CORE_REGS */
	ECOS_REGLIST_BASEPRI,
	ECOS_REGLIST_FPSCR,	/* Following for FPU contexts */
	ECOS_REGLIST_D0,
	ECOS_REGLIST_D1,
	ECOS_REGLIST_D2,
	ECOS_REGLIST_D3,
	ECOS_REGLIST_D4,
	ECOS_REGLIST_D5,
	ECOS_REGLIST_D6,
	ECOS_REGLIST_D7,
	ECOS_REGLIST_D8,
	ECOS_REGLIST_D9,
	ECOS_REGLIST_D10,
	ECOS_REGLIST_D11,
	ECOS_REGLIST_D12,
	ECOS_REGLIST_D13,
	ECOS_REGLIST_D14,
	ECOS_REGLIST_D15
};

#if defined(_EXTENDED_REGLIST)
/* ASCERTAIN: Work is needed to get OpenOCD to request/accept an extended
 * register list with BASEPRI+VFP registers. The target.xml returned by
 * $qXfer:features:read:target.xml:... contains all the VFP information. None of
 * the other RTOS aware source files deal with returning thread specific VFP
 * state (only changed stack offsets for the core registers when FPU state may
 * also be saved).
 *
 * "org.gnu.gdb.arm.m-profile" contains the 17 base register definitions.
 * "org.gnu.gdb.arm.m-system" has BASEPRI as regnum 21 (8-bits wide)
 * "org.gnu.gdb.arm.vfp" has FPSCR and the 16 64-bit registers
 *
 * The src/target/armv7m.c:armv7m_regs[] has the entries and the
 * armv7m_map_id_to_regsel() function in that source maps to the ARM
 * register. It might just be a limitation of the RTOS support API.
 */
# define ECOS_CORTEXM_BASE_NUMREGS (ECOS_REGLIST_BASEPRI + 1);

/* IMPLEMENT: When we enable the support for per-thread non-FPU/FPU register
 * state being returned (since we will want to cache both forms) we could
 * consider dynamically allocating a non-FPU vs FPU stack descriptor structures
 * rather than having pre-defined static vectors as below. The best approach can
 * be decided upon once it is known how OpenOCD will handle RTOS per-thread FPU
 * register support. */

#else /* !_EXTENDED_REGLIST */
# define ECOS_CORTEXM_BASE_NUMREGS (ARMV7M_NUM_CORE_REGS)
#endif /* !_EXTENDED_REGLIST */

/* NOTE: The offsets in this vector are overwritten by the architecture specific
 * layout functions depending on the specific application configuration. The
 * ordering of this vector MUST match eCos_reglist. */
static struct stack_register_offset rtos_ecos_regoff_cortexm[] = {
	{ ARMV7M_R0,      -1, 32 },	/* r0            */
	{ ARMV7M_R1,      -1, 32 },	/* r1            */
	{ ARMV7M_R2,      -1, 32 },	/* r2            */
	{ ARMV7M_R3,      -1, 32 },	/* r3            */
	{ ARMV7M_R4,      -1, 32 },	/* r4            */
	{ ARMV7M_R5,      -1, 32 },	/* r5            */
	{ ARMV7M_R6,      -1, 32 },	/* r6            */
	{ ARMV7M_R7,      -1, 32 },	/* r7            */
	{ ARMV7M_R8,      -1, 32 },	/* r8            */
	{ ARMV7M_R9,      -1, 32 },	/* r9            */
	{ ARMV7M_R10,     -1, 32 },	/* r10           */
	{ ARMV7M_R11,     -1, 32 },	/* r11           */
	{ ARMV7M_R12,     -1, 32 },	/* r12           */
	{ ARMV7M_R13,     -1, 32 },	/* sp            */
	{ ARMV7M_R14,     -1, 32 },	/* lr            */
	{ ARMV7M_PC,      -1, 32 },	/* pc            */
	{ ARMV7M_xPSR,    -1, 32 },	/* xPSR          */
#if defined(_EXTENDED_REGLIST)
	{ ARMV7M_BASEPRI, -1, 32 },	/* BASEPRI       */
	{ ARMV7M_FPSCR,   -1, 32 },	/* FPSCR         */
	{ ARMV7M_D0,      -1, 64 },	/* D0  (S0/S1)   */
	{ ARMV7M_D1,      -1, 64 },	/* D1  (S2/S3)   */
	{ ARMV7M_D2,      -1, 64 },	/* D2  (S4/S5)   */
	{ ARMV7M_D3,      -1, 64 },	/* D3  (S6/S7)   */
	{ ARMV7M_D4,      -1, 64 },	/* D4  (S8/S9)   */
	{ ARMV7M_D5,      -1, 64 },	/* D5  (S10/S11) */
	{ ARMV7M_D6,      -1, 64 },	/* D6  (S12/S13) */
	{ ARMV7M_D7,      -1, 64 },	/* D7  (S14/S15) */
	{ ARMV7M_D8,      -1, 64 },	/* D8  (S16/S17) */
	{ ARMV7M_D9,      -1, 64 },	/* D9  (S18/S19) */
	{ ARMV7M_D10,     -1, 64 },	/* D10 (S20/S21) */
	{ ARMV7M_D11,     -1, 64 },	/* D11 (S22/S23) */
	{ ARMV7M_D12,     -1, 64 },	/* D12 (S24/S25) */
	{ ARMV7M_D13,     -1, 64 },	/* D13 (S26/S27) */
	{ ARMV7M_D14,     -1, 64 },	/* D14 (S28/S29) */
	{ ARMV7M_D15,     -1, 64 },	/* D15 (S30/S31) */
#endif /* _EXTENDED_REGLIST */
};

static struct stack_register_offset rtos_ecos_regoff_arm[] = {
	{ 0,  -1, 32 },		/* r0       */
	{ 1,  -1, 32 },		/* r1       */
	{ 2,  -1, 32 },		/* r2       */
	{ 3,  -1, 32 },		/* r3       */
	{ 4,  -1, 32 },		/* r4       */
	{ 5,  -1, 32 },		/* r5       */
	{ 6,  -1, 32 },		/* r6       */
	{ 7,  -1, 32 },		/* r7       */
	{ 8,  -1, 32 },		/* r8       */
	{ 9,  -1, 32 },		/* r9       */
	{ 10, -1, 32 },		/* r10      */
	{ 11, -1, 32 },		/* r11 (fp) */
	{ 12, -1, 32 },		/* r12 (ip) */
	{ 13, -1, 32 },		/* sp (r13) */
	{ 14, -1, 32 },		/* lr (r14) */
	{ 15, -1, 32 },		/* pc (r15) */
	{ 16, -1, 32 },		/* xPSR     */
};

static struct rtos_register_stacking rtos_eCos_stacking = {
	.stack_registers_size = 0,
	.stack_growth_direction = -1,
	.num_output_registers = 0,
	.calculate_process_stack = NULL,	/* stack_alignment */
	.register_offsets = NULL
};

/* To avoid the run-time cost of matching explicit symbol names we push the
 * lookup offsets to this *manually* maintained enumeration which must match the
 * eCos_symbol_list[] order below. */
enum eCos_symbol_values {
	eCos_VAL_thread_list = 0,
	eCos_VAL_current_thread_ptr,
	eCos_VAL_common_thread_next_off,
	eCos_VAL_common_thread_next_size,
	eCos_VAL_common_thread_state_off,
	eCos_VAL_common_thread_state_size,
	eCos_VAL_common_thread_sleep_off,
	eCos_VAL_common_thread_sleep_size,
	eCos_VAL_common_thread_wake_off,
	eCos_VAL_common_thread_wake_size,
	eCos_VAL_common_thread_id_off,
	eCos_VAL_common_thread_id_size,
	eCos_VAL_common_thread_name_off,
	eCos_VAL_common_thread_name_size,
	eCos_VAL_common_thread_pri_off,
	eCos_VAL_common_thread_pri_size,
	eCos_VAL_common_thread_stack_off,
	eCos_VAL_common_thread_stack_size,
	eCos_VAL_cortexm_thread_saved,
	eCos_VAL_cortexm_ctx_thread_size,
	eCos_VAL_cortexm_ctx_type_off,
	eCos_VAL_cortexm_ctx_type_size,
	eCos_VAL_cortexm_ctx_basepri_off,
	eCos_VAL_cortexm_ctx_basepri_size,
	eCos_VAL_cortexm_ctx_sp_off,
	eCos_VAL_cortexm_ctx_sp_size,
	eCos_VAL_cortexm_ctx_reg_off,
	eCos_VAL_cortexm_ctx_reg_size,
	eCos_VAL_cortexm_ctx_pc_off,
	eCos_VAL_cortexm_ctx_pc_size,
	eCos_VAL_cortexm_val_exception,
	eCos_VAL_cortexm_val_thread,
	eCos_VAL_cortexm_val_interrupt,
	eCos_VAL_cortexm_val_fpu,
	eCos_VAL_cortexm_ctx_fpscr_off,
	eCos_VAL_cortexm_ctx_fpscr_size,
	eCos_VAL_cortexm_ctx_s_off,
	eCos_VAL_cortexm_ctx_s_size,
	eCos_VAL_arm_regsize,
	eCos_VAL_arm_ctx_r0_off,
	eCos_VAL_arm_ctx_r1_off,
	eCos_VAL_arm_ctx_r2_off,
	eCos_VAL_arm_ctx_r3_off,
	eCos_VAL_arm_ctx_r4_off,
	eCos_VAL_arm_ctx_r5_off,
	eCos_VAL_arm_ctx_r6_off,
	eCos_VAL_arm_ctx_r7_off,
	eCos_VAL_arm_ctx_r8_off,
	eCos_VAL_arm_ctx_r9_off,
	eCos_VAL_arm_ctx_r10_off,
	eCos_VAL_arm_ctx_fp_off,
	eCos_VAL_arm_ctx_ip_off,
	eCos_VAL_arm_ctx_sp_off,
	eCos_VAL_arm_ctx_lr_off,
	eCos_VAL_arm_ctx_pc_off,
	eCos_VAL_arm_ctx_cpsr_off,
	eCos_VAL_arm_fpusize,
	eCos_VAL_arm_ctx_fpscr_off,
	eCos_VAL_arm_scount,
	eCos_VAL_arm_ctx_svec_off,
	eCos_VAL_arm_vfpcount,
	eCos_VAL_arm_ctx_vfpvec_off
};

struct symbols {
	const char *name;
	const char * const *target_names; /* non-NULL when for a specific architecture */
	bool optional;
};

#define ECOSSYM(_n, _o, _t) { .name = _n, .optional = (_o), .target_names = _t }

/* Some of offset/size helper symbols are common to all eCos
 * targets. Unfortunately, for historical reasons, some information is in
 * architecture specific namespaces leading to some duplication and a larger
 * vector below. */
static const struct symbols eCos_symbol_list[] = {
	ECOSSYM("Cyg_Thread::thread_list", false, NULL),
	ECOSSYM("Cyg_Scheduler_Base::current_thread", false, NULL),
	/* Following symbols *are* required for generic application-specific
	 * configuration support, but we mark as optional for backwards
	 * compatibility with the previous fixed Cortex-M3 only RTOS plugin
	 * implementation. */
	ECOSSYM("__ecospro_syminfo.off.cyg_thread.list_next", true, NULL),
	ECOSSYM("__ecospro_syminfo.size.cyg_thread.list_next", true, NULL),
	ECOSSYM("__ecospro_syminfo.off.cyg_thread.state", true, NULL),
	ECOSSYM("__ecospro_syminfo.size.cyg_thread.state", true, NULL),
	ECOSSYM("__ecospro_syminfo.off.cyg_thread.sleep_reason", true, NULL),
	ECOSSYM("__ecospro_syminfo.size.cyg_thread.sleep_reason", true, NULL),
	ECOSSYM("__ecospro_syminfo.off.cyg_thread.wake_reason", true, NULL),
	ECOSSYM("__ecospro_syminfo.size.cyg_thread.wake_reason", true, NULL),
	ECOSSYM("__ecospro_syminfo.off.cyg_thread.unique_id", true, NULL),
	ECOSSYM("__ecospro_syminfo.size.cyg_thread.unique_id", true, NULL),
	ECOSSYM("__ecospro_syminfo.off.cyg_thread.name", true, NULL),
	ECOSSYM("__ecospro_syminfo.size.cyg_thread.name", true, NULL),
	ECOSSYM("__ecospro_syminfo.off.cyg_thread.priority", true, NULL),
	ECOSSYM("__ecospro_syminfo.size.cyg_thread.priority", true, NULL),
	ECOSSYM("__ecospro_syminfo.off.cyg_thread.stack_ptr", true, NULL),
	ECOSSYM("__ecospro_syminfo.size.cyg_thread.stack_ptr", true, NULL),
	/* optional Cortex-M: */
	ECOSSYM("__ecospro_syminfo.cortexm.thread.saved", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.size.HAL_SavedRegisters.Thread", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.off.HAL_SavedRegisters.u.thread.type", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.size.HAL_SavedRegisters.u.thread.type", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.off.HAL_SavedRegisters.u.thread.basepri", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.size.HAL_SavedRegisters.u.thread.basepri", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.off.HAL_SavedRegisters.u.thread.sp", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.size.HAL_SavedRegisters.u.thread.sp", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.off.HAL_SavedRegisters.u.thread.r", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.size.HAL_SavedRegisters.u.thread.r", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.off.HAL_SavedRegisters.u.thread.pc", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.size.HAL_SavedRegisters.u.thread.pc", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.value.HAL_SAVEDREGISTERS.EXCEPTION", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.value.HAL_SAVEDREGISTERS.THREAD", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.value.HAL_SAVEDREGISTERS.INTERRUPT", true, target_cortex_m),
	/* optional Cortex-M with H/W FPU configured: */
	ECOSSYM("__ecospro_syminfo.value.HAL_SAVEDREGISTERS.WITH_FPU", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.off.HAL_SavedRegisters.u.thread.fpscr", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.size.HAL_SavedRegisters.u.thread.fpscr", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.off.HAL_SavedRegisters.u.thread.s", true, target_cortex_m),
	ECOSSYM("__ecospro_syminfo.size.HAL_SavedRegisters.u.thread.s", true, target_cortex_m),
	/* optional ARM: */
	ECOSSYM("ARMREG_SIZE", true, target_arm),
	ECOSSYM("armreg_r0", true, target_arm),
	ECOSSYM("armreg_r1", true, target_arm),
	ECOSSYM("armreg_r2", true, target_arm),
	ECOSSYM("armreg_r3", true, target_arm),
	ECOSSYM("armreg_r4", true, target_arm),
	ECOSSYM("armreg_r5", true, target_arm),
	ECOSSYM("armreg_r6", true, target_arm),
	ECOSSYM("armreg_r7", true, target_arm),
	ECOSSYM("armreg_r8", true, target_arm),
	ECOSSYM("armreg_r9", true, target_arm),
	ECOSSYM("armreg_r10", true, target_arm),
	ECOSSYM("armreg_fp", true, target_arm),
	ECOSSYM("armreg_ip", true, target_arm),
	ECOSSYM("armreg_sp", true, target_arm),
	ECOSSYM("armreg_lr", true, target_arm),
	ECOSSYM("armreg_pc", true, target_arm),
	ECOSSYM("armreg_cpsr", true, target_arm),
	/* optional ARM FPU common: */
	ECOSSYM("ARMREG_FPUCONTEXT_SIZE", true, target_arm),
	ECOSSYM("armreg_fpscr", true, target_arm),
	/* optional ARM FPU single-precision: */
	ECOSSYM("ARMREG_S_COUNT", true, target_arm),
	ECOSSYM("armreg_s_vec", true, target_arm),
	/* optional ARM FPU double-precision: */
	ECOSSYM("ARMREG_VFP_COUNT", true, target_arm),
	ECOSSYM("armreg_vfp_vec", true, target_arm),
};

const struct rtos_type eCos_rtos = {
	.name = "eCos",

	.detect_rtos = eCos_detect_rtos,
	.create = eCos_create,
	.update_threads = eCos_update_threads,
	.get_thread_reg_list = eCos_get_thread_reg_list,
	.get_symbol_list_to_lookup = eCos_get_symbol_list_to_lookup,

};

static symbol_address_t ecos_value(struct rtos *rtos, unsigned int idx)
{
	symbol_address_t val = 0;

	if (idx < ARRAY_SIZE(eCos_symbol_list)) {
		val = rtos->symbols[idx].address;
	} else {
		/* We do not terminate, just return 0 in this case. */
		LOG_ERROR("eCos: Invalid symbol index %u", idx);
	}

	return val;
}

/** Escape any XML reserved characters in a string. */
static bool eCos_escape_string(const char *raw, char *out, size_t limit)
{
	static const char *tokens = "<&>\'\"";
	#define XMLENTRY(_c, _s) { .xc = (_c), .rs = (_s), .rlen = (sizeof(_s)-1) }
	static const struct {
		char xc;
		const char *rs;
		size_t rlen;
	} xmlchars[] = {
		XMLENTRY('<', "&lt;"),
		XMLENTRY('&', "&amp;"),
		XMLENTRY('>', "&gt;"),
		XMLENTRY('\'', "&apos;"),
		XMLENTRY('\"', "&quot;")
	};
	bool escaped = false;

	if (out && limit) {
		(void)memset(out, '\0', limit);

		while (raw && *raw && limit) {
			size_t lok = strcspn(raw, tokens);
			if (lok) {
				size_t tocopy;
				tocopy = ((limit < lok) ? limit : lok);
				(void)memcpy(out, raw, tocopy);
				limit -= tocopy;
				out += tocopy;
				raw += lok;
			} else {
				char *fidx = strchr(tokens, *raw);
				if (fidx) {
					uint32_t cidx = (fidx - tokens);
					size_t tocopy = xmlchars[cidx].rlen;
					if (limit < tocopy) {
						break;
					} else {
						escaped = true;
						(void)memcpy(out,
							xmlchars[cidx].rs,
							tocopy);
						limit -= tocopy;
						out += tocopy;
						raw++;
					}
				} else {
					/* Should never happen assuming xmlchars
					 * vector and tokens string match. */
					LOG_ERROR("eCos: Unexpected XML char %c", *raw);
				}
			}
		}
	}

	return escaped;
}

static int eCos_check_app_info(struct rtos *rtos, struct eCos_params *param)
{
	if ((NULL == rtos) || (NULL == param))
		return -1;

	if (param->flush_common) {
		if (LOG_LVL_DEBUG <= debug_level) {
			unsigned int idx;
			for (idx = 0; (idx < ARRAY_SIZE(eCos_symbol_list)); idx++) {
				LOG_DEBUG("eCos: %s 0x%016" PRIX64 " %s",
					(rtos->symbols[idx].optional ? "OPTIONAL" : "        "),
					rtos->symbols[idx].address, rtos->symbols[idx].symbol_name);
			}
		}

		/* If "__ecospro_syminfo.size.cyg_thread.list_next" is non-zero then we
		 * expect all of the generic thread structure symbols to have been
		 * provided. */
		if (0 != ecos_value(rtos, eCos_VAL_common_thread_next_size)) {
			param->pointer_width = ecos_value(rtos, eCos_VAL_common_thread_next_size);
			param->uid_width = ecos_value(rtos, eCos_VAL_common_thread_id_size);
			param->state_width = ecos_value(rtos, eCos_VAL_common_thread_state_size);
			param->thread_stack_offset = ecos_value(rtos, eCos_VAL_common_thread_stack_off);
			param->thread_name_offset = ecos_value(rtos, eCos_VAL_common_thread_name_off);
			param->thread_state_offset = ecos_value(rtos, eCos_VAL_common_thread_state_off);
			param->thread_next_offset = ecos_value(rtos, eCos_VAL_common_thread_next_off);
			param->thread_uniqueid_offset = ecos_value(rtos, eCos_VAL_common_thread_id_off);
		}

		param->stacking_info = NULL;
		param->flush_common = false;
	}

	return ERROR_OK;
}

/* The Cortex-M eCosPro "thread" contexts have a "type" indicator, which tracks
 * the context state of (THREAD | EXCEPTION | INTERRUPT) and whether FPU
 * registers are saved.
 *
 * For thread-aware debugging from GDB we are only interested in THREAD states
 * and so do not need to implement support for INTERRUPT or EXCEPTION thread
 * contexts since this code does not expose those stack contexts via the
 * constructed thread list support. */
static int eCos_stack_layout_cortexm(struct rtos *rtos,
		struct eCos_params *param, int64_t stack_ptr,
		const struct rtos_register_stacking **si)
{
	int retval = ERROR_OK;

	/* CONSIDER: We could return
	 * ecos_value(rtos, eCos_VAL_cortexm_thread_saved) as the actual PC
	 * address of a context switch, with the LR being set to the context PC
	 * field to give a true representation of where the thread switch
	 * occurs. However that would require extending the common
	 * rtos_generic_stack_read() code with suitable support for applying a
	 * supplied value, or just implementing our own version of that code that
	 * can inject data into what is passed onwards to GDB. */

	/* UPDATE: When we can return VFP register state then we will NOT be
	 * basing the cached state on the single param->stacking_info value,
	 * since we will need a different stacking_info structure returned for
	 * each thread type when FPU support is enabled. The use of the single
	 * param->stacking_info is a holder whilst we are limited to the fixed
	 * ARMV7M_NUM_CORE_REGS set of descriptors. */

	if ((NULL == param->stacking_info) &&
		(0 != ecos_value(rtos, eCos_VAL_cortexm_thread_saved)) &&
		(0 != ecos_value(rtos, eCos_VAL_cortexm_val_thread))) {
		unsigned char numoutreg = ECOS_CORTEXM_BASE_NUMREGS;

		rtos_eCos_stacking.stack_registers_size = ecos_value(rtos, eCos_VAL_cortexm_ctx_thread_size);
		rtos_eCos_stacking.calculate_process_stack = rtos_generic_stack_align8;
		rtos_eCos_stacking.register_offsets = rtos_ecos_regoff_cortexm;

		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R0].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x00);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R1].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x04);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R2].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x08);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R3].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x0C);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R4].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x10);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R5].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x14);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R6].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x18);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R7].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x1C);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R8].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x20);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R9].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x24);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R10].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x28);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R11].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x2C);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R12].offset = (ecos_value(rtos, eCos_VAL_cortexm_ctx_reg_off) + 0x30);
		/* Rather than using the stacked eCos_VAL_cortexm_ctx_sp_off
		 * value we force the reported sp to be after the stacked
		 * register context. */
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R13].offset = -2;
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_R14].offset = -1;
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_PC].offset = ecos_value(rtos, eCos_VAL_cortexm_ctx_pc_off);
		rtos_ecos_regoff_cortexm[ECOS_REGLIST_xPSR].offset = -1;

#if defined(_EXTENDED_REGLIST)
		/* For non-FPU configurations we do not bother checking the
		 * thread context type, since for the scheduler thread list it
		 * can only contain THREAD contexts. However we do need to check
		 * it for lazy FPU state if the FPU is configured in the
		 * application. */
		if (0 != ecos_value(rtos, eCos_VAL_cortexm_val_fpu)) {
			symbol_address_t typeoff = ecos_value(rtos, eCos_VAL_cortexm_ctx_type_off);
			uint32_t thread_type = 0;

			retval = target_read_buffer(rtos->target,
					(stack_ptr + typeoff),
					ecos_value(rtos, eCos_VAL_cortexm_ctx_type_size),
					(uint8_t *)&thread_type);

			if (ERROR_OK == retval) {
				const uint32_t fputhread;
				fputhread = (ecos_value(rtos, eCos_VAL_cortexm_val_thread) |
								ecos_value(rtos, eCos_VAL_cortexm_val_fpu));

				if (fputhread == thread_type) {
					if (ecos_value(rtos, eCos_VAL_cortexm_ctx_fpscr_size)) {
						rtos_ecos_regoff_cortexm[ECOS_REGLIST_FPSCR].offset =
							ecos_value(rtos, eCos_VAL_cortexm_ctx_fpscr_off);
						rtos_ecos_regoff_cortexm[ECOS_REGLIST_FPSCR].width_bits =
							(ecos_value(rtos, eCos_VAL_cortexm_ctx_fpscr_size) * 8);
						numoutreg++;

						if ((ecos_value(rtos, eCos_VAL_cortexm_ctx_s_size)) &&
							(0 == (ecos_value(rtos, eCos_VAL_cortexm_ctx_s_size) % 8))) {
							unsigned char soff = ecos_value(rtos, eCos_VAL_cortexm_ctx_s_off);
							unsigned char roff = 0;

							numoutreg = ECOS_REGLIST_D0;

							while (roff < ecos_value(rtos, eCos_VAL_cortexm_ctx_s_size)) {
								rtos_ecos_regoff_cortexm[numoutreg].offset = (soff + roff);
								rtos_ecos_regoff_cortexm[numoutreg].width_bits = 64;
								roff += 0x08;
								numoutreg++;
							}
						}
					} else {
						rtos_ecos_regoff_cortexm[ECOS_REGLIST_FPSCR].offset = -1;
						rtos_ecos_regoff_cortexm[ECOS_REGLIST_FPSCR].width_bits = 32;
					}

					param->stacking_info = &rtos_eCos_stacking;
				} else {
					param->stacking_info = &rtos_eCos_stacking;
				}
			}
		} else {
			param->stacking_info = &rtos_eCos_stacking;
		}
#else /* !_EXTENDED_REGLIST */
		param->stacking_info = &rtos_eCos_stacking;
#endif /* !_EXTENDED_REGLIST */

		/* Common Cortex-M thread register offsets for the current
		 * symbol table: */
		if ((ERROR_OK == retval) && param->stacking_info) {
			if (ECOS_REGLIST_BASEPRI < numoutreg) {
				rtos_ecos_regoff_cortexm[ECOS_REGLIST_BASEPRI].offset =
					ecos_value(rtos, eCos_VAL_cortexm_ctx_basepri_off);
			}

			rtos_eCos_stacking.num_output_registers = numoutreg;
		}
	}

	if (si)
		*si = param->stacking_info;

	return retval;
}

static int eCos_stack_layout_arm(struct rtos *rtos, struct eCos_params *param,
		int64_t stack_ptr, const struct rtos_register_stacking **si)
{
	int retval = ERROR_OK;

	if ((NULL == param->stacking_info) &&
		(0 != ecos_value(rtos, eCos_VAL_arm_regsize))) {
		/* When _EXTENDED_REGLIST is enabled to allow FPU registers to be
		 * returned from a stacked thread context we can check:
		 *		if (0 != ecos_value(rtos, eCos_VAL_arm_fpusize)) { FPU }
		 * for presence of FPU registers in the context. */

		rtos_eCos_stacking.stack_registers_size = ecos_value(rtos, eCos_VAL_arm_regsize);
		rtos_eCos_stacking.num_output_registers = ARRAY_SIZE(rtos_ecos_regoff_arm);
		rtos_eCos_stacking.register_offsets = rtos_ecos_regoff_arm;

		rtos_ecos_regoff_arm[0].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r0_off);
		rtos_ecos_regoff_arm[1].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r1_off);
		rtos_ecos_regoff_arm[2].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r2_off);
		rtos_ecos_regoff_arm[3].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r3_off);
		rtos_ecos_regoff_arm[4].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r4_off);
		rtos_ecos_regoff_arm[5].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r5_off);
		rtos_ecos_regoff_arm[6].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r6_off);
		rtos_ecos_regoff_arm[7].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r7_off);
		rtos_ecos_regoff_arm[8].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r8_off);
		rtos_ecos_regoff_arm[9].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r9_off);
		rtos_ecos_regoff_arm[10].offset = ecos_value(rtos, eCos_VAL_arm_ctx_r10_off);
		rtos_ecos_regoff_arm[11].offset = ecos_value(rtos, eCos_VAL_arm_ctx_fp_off);
		rtos_ecos_regoff_arm[12].offset = ecos_value(rtos, eCos_VAL_arm_ctx_ip_off);
		rtos_ecos_regoff_arm[13].offset = ecos_value(rtos, eCos_VAL_arm_ctx_sp_off);
		rtos_ecos_regoff_arm[14].offset = ecos_value(rtos, eCos_VAL_arm_ctx_lr_off);
		rtos_ecos_regoff_arm[15].offset = ecos_value(rtos, eCos_VAL_arm_ctx_pc_off);
		rtos_ecos_regoff_arm[16].offset = ecos_value(rtos, eCos_VAL_arm_ctx_cpsr_off);

		param->stacking_info = &rtos_eCos_stacking;
	}

	if (si)
		*si = param->stacking_info;

	return retval;
}

/* We see this function called on a new connection, it looks like before and
 * after the "tar rem"/"tar extended-remote". It might be the only point we can
 * decide to cache information (to check if the symbol table has changed). */
static int eCos_update_threads(struct rtos *rtos)
{
	int retval;
	int tasks_found = 0;
	int thread_list_size = 0;
	struct eCos_params *param;

	if (rtos == NULL)
		return -1;

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (struct eCos_params *) rtos->rtos_specific_params;

	if (rtos->symbols == NULL) {
		/* NOTE: We only see this when connecting from GDB the first
		 * time before the application image is loaded. So it is not a
		 * hook for detecting an application change. */
		param->flush_common = true;
		LOG_ERROR("No symbols for eCos");
		return -4;
	}

	retval = eCos_check_app_info(rtos, param);
	if (ERROR_OK != retval)
		return retval;

	if (rtos->symbols[eCos_VAL_thread_list].address == 0) {
		LOG_ERROR("Don't have the thread list head");
		return -2;
	}

	/* determine the number of current threads */
	uint32_t thread_list_head = rtos->symbols[eCos_VAL_thread_list].address;
	uint32_t thread_index;
	target_read_buffer(rtos->target,
		thread_list_head,
		param->pointer_width,
		(uint8_t *) &thread_index);
	uint32_t first_thread = thread_index;

	/* Even if 0==first_thread indicates a system with no defined eCos
	 * threads, instead of early exiting here we fall through the code to
	 * allow the creation of a faked "Current Execution" descriptor as
	 * needed. */

	if (first_thread) {
		/* Since the OpenOCD RTOS support can attempt to obtain thread
		 * information on initial connection when the system *may* have
		 * undefined memory state it is possible for a simple thread count scan
		 * to produce invalid results. To avoid blocking indefinately when
		 * encountering an invalid closed loop we limit the number of threads to
		 * the maximum possible, and if we pass that limit then something is
		 * wrong so treat the system as having no threads defined. */
		do {
			thread_list_size++;
			if (ECOS_MAX_THREAD_COUNT < thread_list_size) {
				/* Treat as "no threads" case: */
				first_thread = 0;
				thread_list_size = 0;
				break;
			}
			retval = target_read_buffer(rtos->target,
					thread_index + param->thread_next_offset,
					param->pointer_width,
					(uint8_t *) &thread_index);
			if (retval != ERROR_OK)
				return retval;
		} while (thread_index != first_thread);
	}

	/* read the current thread id */
	rtos->current_thread = 0;

	uint32_t current_thread_addr;
	retval = target_read_buffer(rtos->target,
			rtos->symbols[eCos_VAL_current_thread_ptr].address,
			param->pointer_width,
			(uint8_t *)&current_thread_addr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Reading active thread address");
		return retval;
	}

	if (current_thread_addr) {
		retval = target_read_buffer(rtos->target,
				current_thread_addr + param->thread_uniqueid_offset,
				param->uid_width,
				(uint8_t *)&rtos->current_thread);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read eCos current thread from target");
			return retval;
		}
	}

	if ((thread_list_size == 0) || (rtos->current_thread == 0)) {
		/* Either : No RTOS threads - there is always at least the current execution though */
		/* OR     : No current thread - all threads suspended - show the current execution
		 * of idling */
		const char tmp_str[] = "Current Execution";
		thread_list_size++;
		tasks_found++;
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * thread_list_size);
		/* 1 is a valid eCos thread id, so we return 0 for this faked
		 * "current" CPU state: */
		rtos->thread_details->threadid = 0;
		rtos->thread_details->exists = true;
		rtos->thread_details->extra_info_str = NULL;
		rtos->thread_details->thread_name_str = malloc(sizeof(tmp_str));
		strcpy(rtos->thread_details->thread_name_str, tmp_str);

		/* Early exit if current CPU state our only "thread": */
		if (thread_list_size == 1) {
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
		uint32_t name_ptr = 0;
		uint32_t prev_thread_ptr;

		/* Save the thread pointer */
		threadid_t thread_id = 0;
		retval = target_read_buffer(rtos->target,
				thread_index + param->thread_uniqueid_offset,
				param->uid_width,
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

		/* Since eCos can have arbitrary C string names we can sometimes
		 * get an internal warning from GDB about "not well-formed
		 * (invalid token)" since the XML post-processing done by GDB on
		 * the OpenOCD returned response containing the thread strings
		 * is not escaped. For example the eCos kernel testsuite
		 * application tm_basic uses the thread name "<<NULL>>" which
		 * will trigger this failure unless escaped. */
		if (tmp_str[0] == '\x00')
			snprintf(tmp_str, ECOS_THREAD_NAME_STR_SIZE, "NoName:[0x%08" PRIX32 "]", thread_index);
		else {
			/* The following is a workaround to avoid any issues
			 * from arbitrary eCos thread names causing GDB/OpenOCD
			 * issues. We limit the escaped thread name passed to
			 * GDB to the same length as the un-escaped just to
			 * avoid overly long strings. */
			char esc_str[ECOS_THREAD_NAME_STR_SIZE];
			bool escaped;
			escaped = eCos_escape_string(tmp_str, esc_str, sizeof(esc_str));
			if (escaped)
				strcpy(tmp_str, esc_str);
		}

		rtos->thread_details[tasks_found].thread_name_str =
			malloc(strlen(tmp_str)+1);
		strcpy(rtos->thread_details[tasks_found].thread_name_str, tmp_str);

		/* Read the thread status */
		int64_t thread_status = 0;
		retval = target_read_buffer(rtos->target,
				thread_index + param->thread_state_offset,
				param->state_width,
				(uint8_t *)&thread_status);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading thread state from eCos target");
			return retval;
		}

		/* The thread_status is a BITMASK */
		char state_desc[21];		/* Enough for "suspended+countsleep\0" maximum */

		if (thread_status & SUSPENDED)
			strcpy(state_desc, "suspended+");
		else
			state_desc[0] = '\0';

		switch (thread_status & ~SUSPENDED) {
		case RUNNING:
			if (thread_index == current_thread_addr)
				strcat(state_desc, "running");
			else if (thread_status & SUSPENDED)
				state_desc[9] = '\0';	/* Drop '+' from "suspended+" */
			else
				strcat(state_desc, "ready");
			break;
		case SLEEPING:
			strcat(state_desc, "sleeping");
			break;
		case SLEEPSET:
		case COUNTSLEEP:
			strcat(state_desc, "counted sleep");
			break;
		case CREATING:
			strcpy(state_desc, "creating");
			break;
		case EXITED:
			strcpy(state_desc, "exited");
			break;
		default:
			strcpy(state_desc, "unknown state");
			break;
		}

		/* For the moment we do not bother decoding the wake reason for the
		 * active "running" thread, but it is useful providing the sleep reason
		 * for stacked threads. */
		int64_t sleep_reason = 0; /* sleep reason */

		if ((thread_index != current_thread_addr) &&
			(0 != ecos_value(rtos, eCos_VAL_common_thread_sleep_size))) {
			retval = target_read_buffer(rtos->target,
				(thread_index + ecos_value(rtos, eCos_VAL_common_thread_sleep_off)),
				ecos_value(rtos, eCos_VAL_common_thread_sleep_size),
				(uint8_t *)&sleep_reason);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading thread sleep reason from eCos target");
				return retval;
			}
			if ((sleep_reason < 0) ||
				((int64_t)ARRAY_SIZE(ecos_thread_reasons) < sleep_reason)) {
				sleep_reason = 0;
			}
		}

		/* We do not display anything for the Cyg_Thread::NONE reason */
		size_t tr_extra = 0;
		const char *reason_desc = NULL;
		if (sleep_reason)
			reason_desc = ecos_thread_reasons[sleep_reason].desc;
		if (reason_desc)
			tr_extra = (2 + strlen(reason_desc) + 1);

		/* Display thread priority if available: */
		int64_t priority = 0;
		size_t pri_extra = 0;
		if (0 != ecos_value(rtos, eCos_VAL_common_thread_pri_size)) {
			retval = target_read_buffer(rtos->target,
				(thread_index + ecos_value(rtos, eCos_VAL_common_thread_pri_off)),
				ecos_value(rtos, eCos_VAL_common_thread_pri_size),
				(uint8_t *)&priority);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading thread priority from eCos target");
				return retval;
			}
			pri_extra = (12 + 20); /* worst-case ", Priority: " */
		}

		size_t eilen = (8 + strlen(state_desc)+tr_extra+pri_extra);
		char *eistr = malloc(eilen);
		if (eistr) {
			int soff = snprintf(eistr, eilen, "State: %s", state_desc);
			if (tr_extra && reason_desc)
				soff += snprintf(&eistr[soff], (eilen - soff), " (%s)", reason_desc);
			if (pri_extra)
				(void)snprintf(&eistr[soff], (eilen - soff), ", Priority: %" PRId64 "", priority);
		}
		rtos->thread_details[tasks_found].extra_info_str = eistr;

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
	return ERROR_OK;
}

static int eCos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	struct eCos_params *param;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (struct eCos_params *) rtos->rtos_specific_params;

	retval = eCos_check_app_info(rtos, param);
	if (ERROR_OK != retval)
		return retval;

	/* We can get memory access errors reported by this function on
	 * re-connecting to a board with stale thread information in memory. The
	 * initial eCos_update_threads() is called twice and may read
	 * stale/invalid information depending on the memory state. This happens
	 * as part of the "target remote" connection so cannot be avoided by GDB
	 * scripting. It is not critical and allowing the application to run and
	 * initialise its BSS etc. will allow correct thread and register
	 * information to be obtained. This really only affects debug sessions
	 * where "info thr" is used before the initial run-time initialisation
	 * has occurred. */

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
				param->uid_width,
				(uint8_t *)&id);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading unique id from eCos thread 0x%08" PRIX32 "", thread_index);
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

		if (0 == stack_ptr) {
			LOG_ERROR("NULL stack pointer in thread %" PRIu64 "", thread_id);
			return -5;
		}

		const struct rtos_register_stacking *stacking_info = NULL;
		if (param->target_stack_layout) {
			retval = (param->target_stack_layout)(rtos, param, stack_ptr, &stacking_info);
			if (ERROR_OK != retval) {
				LOG_ERROR("Error reading stack layout for eCos thread");
				return retval;
			}
		}
		if (NULL == stacking_info)
			stacking_info = &rtos_eCos_Cortex_M3_stacking;

		return rtos_generic_stack_read(rtos->target,
			stacking_info,
			stack_ptr,
			reg_list,
			num_regs);
	}

	return -1;
}

/* NOTE: This is only called once when the first GDB connection is made to
 * OpenOCD and not on subsequent connections (when the application symbol table
 * may have changed, affecting the offsets of critical fields and the stacked
 * context shape). */
static int eCos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(eCos_symbol_list), sizeof(struct symbol_table_elem));

	/* If the target reference was passed into this function we could limit
	 * the symbols we need to lookup to the target->type->name based
	 * range. For the moment we need to provide a single vector with all of
	 * the symbols across all of the supported architectures. */
	for (i = 0; i < ARRAY_SIZE(eCos_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = eCos_symbol_list[i].name;
		(*symbol_list)[i].optional = eCos_symbol_list[i].optional;
	}

	return 0;
}

/* NOTE: Only called by rtos.c:rtos_qsymbol() when auto-detecting the RTOS. If
 * the target configuration uses the explicit "-rtos" config option then this
 * detection routine is NOT called. */
static bool eCos_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols != NULL) &&
			(target->rtos->symbols[eCos_VAL_thread_list].address != 0)) {
		/* looks like eCos */
		return true;
	}
	return false;
}

extern int rtos_thread_packet(struct connection *connection,
		const char *packet, int packet_size);

/* Since we should never have 0 as a valid eCos thread ID we use $Hg0 as the
 * indicator of a new session as regards flushing any cached state. */
static int eCos_packet_hook(struct connection *connection,
		const char *packet, int packet_size)
{
	int64_t current_threadid;

	if (packet[0] == 'H' && packet[1] == 'g') {
		(void)sscanf(packet, "Hg%16" SCNx64, &current_threadid);
		if (0 == current_threadid) {
			struct target *target = get_target_from_connection(connection);
			if (target && target->rtos && target->rtos->rtos_specific_params) {
				struct eCos_params *param;
				param = (struct eCos_params *)target->rtos->rtos_specific_params;
				param->flush_common = true;
			}
		}
	}

	return rtos_thread_packet(connection, packet, packet_size);
}

/* Called at start of day when eCos detected or specified in config file. */
static int eCos_create(struct target *target)
{
	unsigned int i = 0;
	bool matched = false;

	while (i < ECOS_NUM_PARAMS) {
		const char * const *tnames = eCos_params_list[i].target_names;
		while (*tnames) {
			if (0 == strcmp(*tnames, target->type->name)) {
				/* LOG_DEBUG("eCos: matched target \"%s\"", target->type->name); */
				matched = true;
				break;
			}
			tnames++;
		}
		if (matched)
			break;
		i++;
	}
	if (i >= ECOS_NUM_PARAMS) {
		LOG_ERROR("Could not find target in eCos compatibility list");
		return -1;
	}

	target->rtos->rtos_specific_params = (void *) &eCos_params_list[i];
	eCos_params_list[i].flush_common = true;
	eCos_params_list[i].stacking_info = NULL;
	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;

	/* We use the $Hg0 packet as a new GDB connection "start-of-day" hook to
	 * force a re-cache of information. It is possible for a single OpenOCD
	 * session to be connected to a target with multiple GDB debug sessions
	 * started/stopped. With eCos it is possible for those GDB sessions to
	 * present applications with different offsets within a thread
	 * descriptor for fields used by this module, and for the stacked
	 * context within the connected target architecture to differ between
	 * applications and even between threads in a single application. So we
	 * need to ensure any information we cache is flushed on an application
	 * change, and GDB referencing an invalid eCos thread ID (0) is a good
	 * enough point, since we can accept the re-cache hit if that packet
	 * appears during an established session, whilst benefiting from not
	 * re-loading information on every update_threads or get_thread_reg_list
	 * call. */
	target->rtos->gdb_thread_packet = eCos_packet_hook;
	/* We do not currently use the target->rtos->gdb_target_for_threadid
	 * hook. */

	return 0;
}

/* Local Variables:  */
/* mode: c           */
/* tab-width: 4      */
/* End:              */
