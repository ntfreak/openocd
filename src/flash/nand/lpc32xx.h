/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
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

#ifndef OPENOCD_FLASH_NAND_LPC32XX_H
#define OPENOCD_FLASH_NAND_LPC32XX_H

enum lpc32xx_selected_controller {
	LPC32XX_NO_CONTROLLER,
	LPC32XX_MLC_CONTROLLER,
	LPC32XX_SLC_CONTROLLER,
};

struct lpc32xx_nand_controller {
	int osc_freq;
	enum lpc32xx_selected_controller selected_controller;
	int sw_write_protection;
	uint32_t sw_wp_lower_bound;
	uint32_t sw_wp_upper_bound;
};

#endif /* OPENOCD_FLASH_NAND_LPC32XX_H */
