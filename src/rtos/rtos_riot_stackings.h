/***************************************************************************
 *   Copyright (C) 2015 by Daniel Krebs                                    *
 *   Daniel Krebs - github@daniel-krebs.net                                *
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
 ***************************************************************************/

#ifndef OPENOCD_RTOS_RIOT_STACKINGS_H
#define OPENOCD_RTOS_RIOT_STACKINGS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"

extern const struct rtos_register_stacking rtos_riot_Cortex_M0_stacking;
extern const struct rtos_register_stacking rtos_riot_Cortex_M34_stacking;
#endif	/* ifndef OPENOCD_RTOS_RIOT_STACKINGS_H */
