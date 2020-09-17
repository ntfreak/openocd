/*
 * Copyright (C) 2016-2017 by Marc Schink <dev@zapb.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OPENOCD_SERVER_RTT_SERVER_H
#define OPENOCD_SERVER_RTT_SERVER_H

#include <helper/command.h>

int rtt_server_register_commands(struct command_context *ctx);

#endif /* OPENOCD_SERVER_RTT_SERVER_H */
