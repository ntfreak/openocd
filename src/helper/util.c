/***************************************************************************
 *   Copyright (C) 2010 by Øyvind Harboe                                   *
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

/* this file contains various functionality useful to standalone systems */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "time_support.h"

static int jim_util_ms(Jim_Interp *interp,
	int argc,
	Jim_Obj * const *argv)
{
	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "ls ?dir?");
		return JIM_ERR;
	}

	/* Cast from 64 to 32 bit int works for 2's-compliment
	 * when calculating differences*/
	Jim_SetResult(interp, Jim_NewIntObj(interp, (int)timeval_ms()));

	return JIM_OK;
}

static const struct command_registration util_command_handlers[] = {
	/* jim handlers */
	{
		.name = "ms",
		.mode = COMMAND_ANY,
		.jim_handler = jim_util_ms,
		.help =
			"Returns ever increasing milliseconds. Used to calculate differences in time.",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

int util_init(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, util_command_handlers);
}
