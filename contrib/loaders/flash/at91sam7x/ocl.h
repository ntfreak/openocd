/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
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
#ifndef OCL_H
#define OCL_H

/* command/response mask */
#define OCL_CMD_MASK 0xFFFF0000L

/* commands */
#define OCL_FLASH_BLOCK 0x0CFB0000L
#define OCL_ERASE_BLOCK 0x0CEB0000L
#define OCL_ERASE_ALL 0x0CEA0000L
#define OCL_PROBE 0x0CBE0000L

/* responses */
#define OCL_CMD_DONE 0x0ACD0000L
#define OCL_CMD_ERR 0x0ACE0000L
#define OCL_CHKS_FAIL 0x0ACF0000L
#define OCL_BUFF_OVER 0x0AB00000L

#define OCL_CHKS_INIT 0xC100CD0CL

#endif /* OCL_H */
