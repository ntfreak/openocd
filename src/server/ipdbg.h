/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright (C) 2020 by Daniel Anselmi <danselmi@gmx.ch> */

#ifndef OPENOCD_IPDBG_IPDBG_H
#define OPENOCD_IPDBG_IPDBG_H

#include <helper/command.h>

int ipdbg_register_commands(struct command_context *cmd_ctx);

#endif /* OPENOCD_IPDBG_IPDBG_H */
