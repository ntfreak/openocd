/***************************************************************************
 *   Copyright (C) 2019 by Mete Balci					   *
 *									   *
 *   SPDX-License-Identifier: GPL-2.0-or-later                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_AARCH64_SYSTEM_REGISTERS_H
#define OPENOCD_TARGET_AARCH64_SYSTEM_REGISTERS_H

struct aarch64_system_register_field {
	const char *name;
	uint8_t msb;
	uint8_t lsb;
};

struct aarch64_system_register {
	const char *short_name;
	const char *long_name;
	const char *op0;
	const char *op1;
	const char *crn;
	const char *crm;
	const char *op2;
	uint32_t encoding;
	struct aarch64_system_register_field fields[64];
};

#include "aarch64_system_registers_defs.h"

#endif /* OPENOCD_TARGET_AARCH64_SYSTEM_REGISTERS_H */
