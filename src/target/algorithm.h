/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef OPENOCD_TARGET_ALGORITHM_H
#define OPENOCD_TARGET_ALGORITHM_H

#include "helper/types.h"
#include "helper/replacements.h"

enum param_direction {
	PARAM_IN,
	PARAM_OUT,
	PARAM_IN_OUT
};

struct mem_param {
	target_addr_t address;
	uint32_t size;
	uint8_t *value;
	enum param_direction direction;
};

struct reg_param {
	const char *reg_name;
	uint32_t size;
	uint8_t *value;
	enum param_direction direction;
};

void init_mem_param(struct mem_param *param,
		uint32_t address, uint32_t size, enum param_direction dir);
void destroy_mem_param(struct mem_param *param);

void init_reg_param(struct reg_param *param,
		char *reg_name, uint32_t size, enum param_direction dir);
void destroy_reg_param(struct reg_param *param);

#endif /* OPENOCD_TARGET_ALGORITHM_H */
