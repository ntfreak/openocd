/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MIPS64 generic target support                                         *
 *
 * Copyright (C) 2014 by Andrey Sidorov <anysidorov@gmail.com>
 * Copyright (C) 2014 by Aleksey Kuleshov <rndfax@yandex.ru>
 * Copyright (C) 2014-2019 by Peter Mamonov <pmamonov@gmail.com>
 *
 * Based on the work of:
 *     Copyright (C) 2008 by Spencer Oliver
 *     Copyright (C) 2008 by David T.L. Wong
 */

#ifndef OPENOCD_TARGET_MIPS_MIPS64_H
#define OPENOCD_TARGET_MIPS_MIPS64_H

#include "helper/types.h"

struct mips_mips64_common {
	int common_magic;
	struct mips64_common mips64_common;
};

#endif /* OPENOCD_TARGET_MIPS_MIPS64_H */
