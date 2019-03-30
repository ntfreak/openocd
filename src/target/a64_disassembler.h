/***************************************************************************
 *   Copyright (C) 2019 by Mete Balci                                      *
 *   metebalci@gmail.com                                                   *
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

#ifndef OPENOCD_TARGET_AARCH64_DISASSEMBLER_H
#define OPENOCD_TARGET_AARCH64_DISASSEMBLER_H

#include "target.h"

int a64_disassemble(
		struct command_invocation *cmd,
		struct target *target,
		target_addr_t address,
		size_t count);

#endif /* OPENOCD_TARGET_AARCH64_DISASSEMBLER_H */
