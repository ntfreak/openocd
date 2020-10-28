/*
 * Copyright(c) 2015-2016 Intel Corporation.
 *
 * Jessica Gomez (jessica.gomez.hernandez@intel.com)
 * Ivan De Cesaris (ivan.de.cesaris@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information:
 * Intel Corporation
 */

/*
 * @file
 * Debugger for Intel Quark D20xx
 * The CPU TAP (Lakemont TAP) is used for software debug and the CLTAP is
 * used for SoC level operations.
 *
 * Reference document:
 * Intel Quark microcontroller D2000 Debug Operations (web search for doc num 333241)
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>

#include "target.h"
#include "target_type.h"
#include "breakpoints.h"
#include "lakemont.h"
#include "x86_32_common.h"

static int quark_d20xx_target_create(struct target *t, Jim_Interp *interp)
{
	struct x86_32_common *x86_32 = calloc(1, sizeof(struct x86_32_common));
	if (x86_32 == NULL) {
		LOG_ERROR("%s out of memory", __func__);
		return ERROR_FAIL;
	}
	x86_32_common_init_arch_info(t, x86_32);
	lakemont_init_arch_info(t, x86_32);
	x86_32->core_type = LMT3_5;
	return ERROR_OK;
}

static int quark_d20xx_init_target(struct command_context *cmd_ctx, struct target *t)
{
	return lakemont_init_target(cmd_ctx, t);
}

static int quark_d20xx_reset_deassert(struct target *t)
{
	int retval;

	/* Can't detect if a warm reset happened while halted but we can make the
	 * openocd and target state consistent here if in probe mode already
	 */
	if (!check_not_halted(t)) {
		retval = lakemont_update_after_probemode_entry(t);
		if (retval != ERROR_OK) {
			LOG_ERROR("%s core state update fail", __func__);
			return retval;
		}
		/* resume target if reset mode is run */
		if (!t->reset_halt) {
			retval = lakemont_resume(t, 1, 0, 0, 0);
			if (retval != ERROR_OK) {
				LOG_ERROR("%s could not resume target", __func__);
				return retval;
			}
		}
	}

	return ERROR_OK;
}

struct target_type quark_d20xx_target = {
	.name = "quark_d20xx",
	.target_create = quark_d20xx_target_create,
	.init_target = quark_d20xx_init_target,
	/* lakemont probemode specific code */
	.poll = lakemont_poll,
	.arch_state = lakemont_arch_state,
	.halt = lakemont_halt,
	.resume = lakemont_resume,
	.step = lakemont_step,
	.assert_reset = lakemont_reset_assert,
	.deassert_reset = quark_d20xx_reset_deassert,
	/* common x86 code */
	.commands = x86_32_command_handlers,
	.get_gdb_reg_list = x86_32_get_gdb_reg_list,
	.read_memory = x86_32_common_read_memory,
	.write_memory = x86_32_common_write_memory,
	.add_breakpoint = x86_32_common_add_breakpoint,
	.remove_breakpoint = x86_32_common_remove_breakpoint,
	.add_watchpoint = x86_32_common_add_watchpoint,
	.remove_watchpoint = x86_32_common_remove_watchpoint,
	.virt2phys = x86_32_common_virt2phys,
	.read_phys_memory = x86_32_common_read_phys_mem,
	.write_phys_memory = x86_32_common_write_phys_mem,
	.mmu = x86_32_common_mmu,
};
