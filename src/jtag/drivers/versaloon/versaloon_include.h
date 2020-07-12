/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian <SimonQian@SimonQian.com>            *
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

#ifndef OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_INCLUDE_H
#define OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_INCLUDE_H

/* This file is used to include different header and macros */
/* according to different platform */
#include <jtag/interface.h>
#include <jtag/commands.h>

#define PARAM_CHECK							1

#define sleep_ms(ms)						jtag_sleep((ms) * 1000)
#define dimof(arr)							(sizeof(arr) / sizeof((arr)[0]))
#define TO_STR(name)						#name

#define RESULT								int
#define LOG_BUG								LOG_ERROR

/* Common error messages */
#define ERRMSG_NOT_ENOUGH_MEMORY			"Lack of memory."
#define ERRCODE_NOT_ENOUGH_MEMORY			ERROR_FAIL

#define ERRMSG_INVALID_VALUE				"%d is invalid for %s."
#define ERRMSG_INVALID_INDEX				"Index %d is invalid for %s."
#define ERRMSG_INVALID_USAGE				"Invalid usage of %s"
#define ERRMSG_INVALID_TARGET				"Invalid %s"
#define ERRMSG_INVALID_PARAMETER			"Invalid parameter of %s."
#define ERRMSG_INVALID_INTERFACE_NUM		"invalid interface %d"
#define ERRMSG_INVALID_BUFFER				"Buffer %s is not valid."
#define ERRCODE_INVALID_BUFFER				ERROR_FAIL
#define ERRCODE_INVALID_PARAMETER			ERROR_FAIL

#define ERRMSG_NOT_SUPPORT_BY				"%s is not supported by %s."

#define ERRMSG_FAILURE_OPERATION			"Fail to %s."
#define ERRMSG_FAILURE_OPERATION_MESSAGE	"Fail to %s, %s"
#define ERRCODE_FAILURE_OPERATION			ERROR_FAIL

#define GET_U16_MSBFIRST(p)			(((*((uint8_t *)(p) + 0)) << 8) | \
									((*((uint8_t *)(p) + 1)) << 0))
#define GET_U32_MSBFIRST(p)			(((*((uint8_t *)(p) + 0)) << 24) | \
									((*((uint8_t *)(p) + 1)) << 16) | \
									((*((uint8_t *)(p) + 2)) << 8) | \
									((*((uint8_t *)(p) + 3)) << 0))
#define GET_U16_LSBFIRST(p)			(((*((uint8_t *)(p) + 0)) << 0) | \
									((*((uint8_t *)(p) + 1)) << 8))
#define GET_U32_LSBFIRST(p)			(((*((uint8_t *)(p) + 0)) << 0) | \
									((*((uint8_t *)(p) + 1)) << 8) | \
									((*((uint8_t *)(p) + 2)) << 16) | \
									((*((uint8_t *)(p) + 3)) << 24))

#define SET_U16_MSBFIRST(p, v)		\
	do {\
		*((uint8_t *)(p) + 0) = (((uint16_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint16_t)(v)) >> 0) & 0xFF;\
	} while (0)
#define SET_U32_MSBFIRST(p, v)		\
	do {\
		*((uint8_t *)(p) + 0) = (((uint32_t)(v)) >> 24) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint32_t)(v)) >> 16) & 0xFF;\
		*((uint8_t *)(p) + 2) = (((uint32_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 3) = (((uint32_t)(v)) >> 0) & 0xFF;\
	} while (0)
#define SET_U16_LSBFIRST(p, v)		\
	do {\
		*((uint8_t *)(p) + 0) = (((uint16_t)(v)) >> 0) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint16_t)(v)) >> 8) & 0xFF;\
	} while (0)
#define SET_U32_LSBFIRST(p, v)		\
	do {\
		*((uint8_t *)(p) + 0) = (((uint32_t)(v)) >> 0) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint32_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 2) = (((uint32_t)(v)) >> 16) & 0xFF;\
		*((uint8_t *)(p) + 3) = (((uint32_t)(v)) >> 24) & 0xFF;\
	} while (0)

#define GET_LE_U16(p)				GET_U16_LSBFIRST(p)
#define GET_LE_U32(p)				GET_U32_LSBFIRST(p)
#define GET_BE_U16(p)				GET_U16_MSBFIRST(p)
#define GET_BE_U32(p)				GET_U32_MSBFIRST(p)
#define SET_LE_U16(p, v)			SET_U16_LSBFIRST(p, v)
#define SET_LE_U32(p, v)			SET_U32_LSBFIRST(p, v)
#define SET_BE_U16(p, v)			SET_U16_MSBFIRST(p, v)
#define SET_BE_U32(p, v)			SET_U32_MSBFIRST(p, v)

#endif /* OPENOCD_JTAG_DRIVERS_VERSALOON_VERSALOON_INCLUDE_H */
