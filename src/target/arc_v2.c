/***************************************************************************
 *   Copyright (C) 2013-2015,2019 Synopsys, Inc.                           *
 *   Frank Dols <frank.dols@synopsys.com>                                  *
 *   Mischa Jonker <mischa.jonker@synopsys.com>                            *
 *   Anton Kolesov <anton.kolesov@synopsys.com>                            *
 *   Evgeniy Didin <didin@synopsys.com>
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arc.h"

static int arc_v2_init_target(struct command_context *cmd_ctx, struct target *target)
{
  /* Will be uncommented in register introdution patch. */
	CHECK_RETVAL(arc_build_reg_cache(target));
	CHECK_RETVAL(arc_build_bcr_reg_cache(target));
	target->debug_reason = DBG_REASON_DBGRQ;
	return ERROR_OK;
}

static int arc_v2_target_create(struct target *target, Jim_Interp *interp)
{
  struct arc_common *arc = calloc(1, sizeof(struct arc_common));

  LOG_DEBUG("Entering");
  CHECK_RETVAL(arc_init_arch_info(target, arc, target->tap));

  return ERROR_OK;
}


/* ARC v2 target */
struct target_type arcv2_target = {
	.name = "arcv2",

	.poll =	arc_poll,

	.arch_state = arc_arch_state,

	/* TODO That seems like something similiar to metaware hostlink, so perhaps
	 * we can exploit this in the future. */
	.target_request_data = NULL,

	.halt = arc_halt,
	.resume = arc_resume,
	.step = NULL,

	.assert_reset = arc_assert_reset,
	.deassert_reset = arc_deassert_reset,

	/* TODO Implement soft_reset_halt */
	.soft_reset_halt = NULL,

	.get_gdb_reg_list = arc_get_gdb_reg_list,

	.read_memory = arc_mem_read,
	.write_memory = arc_mem_write,
	.checksum_memory = NULL,
	.blank_check_memory = NULL,

	.add_breakpoint = NULL,
	.add_context_breakpoint = NULL,
	.add_hybrid_breakpoint = NULL,
	.remove_breakpoint = NULL,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,
	.hit_watchpoint = NULL,

	.run_algorithm = NULL,
	.start_algorithm = NULL,
	.wait_algorithm = NULL,

	.commands = arc_monitor_command_handlers,

	.target_create = arc_v2_target_create,
	.init_target = arc_v2_init_target,
	.examine = arc_examine,

	.virt2phys = NULL,
	.read_phys_memory = NULL,
	.write_phys_memory = NULL,
	.mmu = NULL,
};
