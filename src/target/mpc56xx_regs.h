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

#ifndef MPC56XX_REGS
#define MPC56XX_REGS

int mpc56xx_jtag_read_regs(struct mpc5xxx_jtag *jtag_info, uint32_t *regs, uint32_t *saved_ctl);
int mpc56xx_jtag_write_regs(struct mpc5xxx_jtag *jtag_info, uint32_t *regs);
int mpc56xx_read_reg(struct mpc5xxx_jtag *jtag_info, int reg, uint32_t *val);
int mpc56xx_write_reg(struct mpc5xxx_jtag *jtag_info, int reg, uint32_t val);
int mpc56xx_write_spr(struct mpc5xxx_jtag *jtag_info, int reg, uint32_t val);
int mpc56xx_read_spr(struct mpc5xxx_jtag *jtag_info, int reg, uint32_t *val) ;

#endif /* MPC56XX_REGS */
