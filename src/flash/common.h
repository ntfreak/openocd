/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
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

#ifndef OPENOCD_FLASH_COMMON_H
#define OPENOCD_FLASH_COMMON_H

#include <helper/log.h>
#include <helper/replacements.h>

/**
 * Parses the optional '.index' portion of a flash bank identifier.
 * @param name The desired driver name, passed by the user.
 * @returns The parsed index request, or 0 if not present.  If the
 * name provides a suffix but it does not parse as an unsigned integer,
 * the routine returns ~0U.  This will prevent further matching.
 */
unsigned get_flash_name_index(const char *name);
/**
 * Attempt to match the @c expected name with the @c name of a driver.
 * @param name The name of the driver (from the bank's device structure).
 * @param expected The expected driver name, passed by the user.
 */
bool flash_driver_name_matches(const char *name, const char *expected);

#define ERROR_FLASH_BANK_INVALID			(-900)
#define ERROR_FLASH_SECTOR_INVALID			(-901)
#define ERROR_FLASH_OPERATION_FAILED		(-902)
#define ERROR_FLASH_DST_OUT_OF_BANK			(-903)
#define ERROR_FLASH_DST_BREAKS_ALIGNMENT	(-904)
#define ERROR_FLASH_BUSY					(-905)
#define ERROR_FLASH_SECTOR_NOT_ERASED		(-906)
#define ERROR_FLASH_BANK_NOT_PROBED			(-907)
#define ERROR_FLASH_OPER_UNSUPPORTED		(-908)
#define ERROR_FLASH_PROTECTED			(-909)

#endif /* OPENOCD_FLASH_COMMON_H */
