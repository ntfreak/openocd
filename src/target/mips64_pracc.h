/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Support for processors implementing MIPS64 instruction set
 *
 * Copyright (C) 2014 by Andrey Sidorov <anysidorov@gmail.com>
 * Copyright (C) 2014 by Aleksey Kuleshov <rndfax@yandex.ru>
 * Copyright (C) 2014-2019 by Peter Mamonov <pmamonov@gmail.com>
 *
 * Based on the work of:
 *     Copyright (C) 2008 by Spencer Oliver
 *     Copyright (C) 2008 by David T.L. Wong
 *     Copyright (C) 2010 by Konstantin Kostyukhin, Nikolay Shmyrev
 */

#ifndef OPENOCD_TARGET_MIPS64_PRACC_H
#define OPENOCD_TARGET_MIPS64_PRACC_H

#include "mips_ejtag.h"

#define MIPS64_PRACC_TEXT		0xffffffffFF200200ull

#define MIPS64_PRACC_STACK		0xffffffffFF204000ull
#define MIPS64_PRACC_PARAM_IN		0xffffffffFF201000ull
#define MIPS64_PRACC_PARAM_IN_SIZE	0x1000
#define MIPS64_PRACC_PARAM_OUT		(MIPS64_PRACC_PARAM_IN + MIPS64_PRACC_PARAM_IN_SIZE)
#define MIPS64_PRACC_PARAM_OUT_SIZE	0x1000

#undef UPPER16
#undef LOWER16
#define UPPER16(v) ((uint32_t)((v >> 16) & 0xFFFF))
#define LOWER16(v) ((uint32_t)(v & 0xFFFF))
#define MIPS64_PRACC_FASTDATA_AREA		0xffffffffFF200000
#define MIPS64_PRACC_FASTDATA_SIZE		16
#define MIPS64_FASTDATA_HANDLER_SIZE	0x80

/* FIXME: 16-bit NEG */
#undef NEG16
#define NEG16(v) ((uint32_t)(((~(v)) + 1) & 0xFFFF))

#define MIPS64_PRACC_ADDR_STEP 4
#define MIPS64_PRACC_DATA_STEP 8

int mips64_pracc_read_mem(struct mips_ejtag *ejtag_info, uint64_t addr, unsigned size, unsigned count, void *buf);
int mips64_pracc_write_mem(struct mips_ejtag *ejtag_info, uint64_t addr, unsigned size, unsigned count, void *buf);

int mips64_pracc_read_regs(struct mips_ejtag *ejtag_info, uint64_t *regs);
int mips64_pracc_write_regs(struct mips_ejtag *ejtag_info, uint64_t *regs);

int mips64_pracc_exec(struct mips_ejtag *ejtag_info,
		      unsigned code_len, const uint32_t *code,
		      unsigned num_param_in, uint64_t *param_in,
		      unsigned num_param_out, uint64_t *param_out);

int mips64_pracc_fastdata_xfer(struct mips_ejtag *ejtag_info,
			       struct working_area *source,
			       bool write_t, uint64_t addr,
			       unsigned count, uint64_t *buf);

#endif /* OPENOCD_TARGET_MIPS64_PRACC_H */
