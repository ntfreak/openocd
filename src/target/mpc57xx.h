/***************************************************************************
 *   Copyright (C) 2017 by James Murray <james@nscc.info                   *
 *   Based on code:                                                        *
 *       Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>   *
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

#ifndef MPC57XX
#define MPC57XX

struct target;

#define MPC57XX_COMMON_MAGIC	0x1834601d /* Set as MPC5746R JTAG ID - is this acceptable? */

#define MPC57XX_START_OF_SRAM	0x40008000 /* This needs to come from a config var */
#define MPC57XX_SIZE_OF_SRAM	0x00038000 /* 224k This needs to come from a config var */

#define MPC57XX_FLASH_PFCR1	0xfc030000
#define MPC57XX_FLASH_PFCR2	0xfc030004

#define MPC57XX_FMPLL	0xc3f80000
#define MPC57XX_FMPLL_SYNCR	(MPC57XX_FMPLL+0)
#define MPC57XX_FMPLL_SYNSR	(MPC57XX_FMPLL+4)
#define MPC57XX_FMPLL_ESYNCR1	(MPC57XX_FMPLL+8)
#define MPC57XX_FMPLL_ESYNCR2	(MPC57XX_FMPLL+0xc)
#define MPC57XX_FMPLL_SYNFMRR	(MPC57XX_FMPLL+0x18)

int mpc57xx_save_context(struct target *target);
int mpc57xx_restore_context(struct target *target);

#endif	/*MPC5634*/
