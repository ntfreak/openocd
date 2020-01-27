/***************************************************************************
 *   Copyright (C) 2013-2014,2019-2020 Synopsys, Inc.                      *
 *   Frank Dols <frank.dols@synopsys.com>                                  *
 *   Anton Kolesov <anton.kolesov@synopsys.com>                            *
 *   Evgeniy Didin <didin@synopsys.com>                                    *
 *                                                                         *
 *   SPDX-License-Identifier: GPL-2.0-or-later                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARC_MEM_H
#define OPENOCD_TARGET_ARC_MEM_H

/* ----- Exported functions ------------------------------------------------ */

int arc_mem_read(struct target *target, target_addr_t address, uint32_t size,
	uint32_t count, uint8_t *buffer);
int arc_mem_write(struct target *target, target_addr_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer);


#endif /* OPENOCD_TARGET_ARC_MEM_H */
