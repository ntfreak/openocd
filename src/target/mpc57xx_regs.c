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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "jtag/jtag.h"
#include "mpc5xxx_jtag.h"
#include "mpc5xxx.h"
#include "mpc57xx_jtag.h"
#include "mpc57xx_regs.h"

static int mpc57xx_read_gpr(struct mpc5xxx_jtag *jtag_info, int reg,
		uint32_t *val)
{
	uint32_t opcode;

	if ((reg < 0) || (reg > 31)) {
		LOG_ERROR("Invalid GP register %d requested!", reg);
		printf("Invalid GP register %d requested!\n", reg);
		*val = 0;
		return ERROR_FAIL;
	}
	opcode = 0x60000000 | (reg << 16) | (reg << 21); /* ori Rx, Rx, 0 */
	return mpc57xx_exec_inst(jtag_info, opcode, 0, val, 0);
}

static int mpc57xx_write_gpr(struct mpc5xxx_jtag *jtag_info, int reg,
		uint32_t val)
{
	uint32_t val2, opcode;

	if ((reg < 0) || (reg > 31)) {
		LOG_ERROR("Invalid GP register %d requested!", reg);
		printf("Invalid GP register %d requested!\n", reg);
		return ERROR_FAIL;
	}

	opcode = 0x60000000 | (reg << 16) | (reg << 21); /* ori Rx, Rx, 0 */
	return mpc57xx_exec_inst(jtag_info, opcode, val, &val2, MPC57XX_EI_VAL);
}

int mpc57xx_read_spr(struct mpc5xxx_jtag *jtag_info, int reg,
		uint32_t *val)
{
	int retval;
	uint32_t opcode, r31;

	if ((reg < 0) || (reg > 1023)) {
		LOG_ERROR("Invalid SP register %d requested!", reg);
		printf("Invalid SP register %d requested!\n", reg);
		*val = 0;
		return ERROR_FAIL;
	}
	/* Preserve R31 */
	retval = mpc57xx_read_gpr(jtag_info, MPC5XXX_REG_R31, &r31);
	if (retval)
		return retval;

	/* Transfer special purpose register to R31 */
	opcode = 0x7fe002a6 | ((reg & 0x1f) << 16) | ((reg & 0x3e0) << 6); /* mpspr r31, reg */
	retval = mpc57xx_exec_inst(jtag_info, opcode, 0, val, 0);

	/* Restore R31 */
	return mpc57xx_write_gpr(jtag_info, MPC5XXX_REG_R31, r31);
}

int mpc57xx_write_spr(struct mpc5xxx_jtag *jtag_info, int reg,
		uint32_t val)
{
	uint32_t opcode;

	if ((reg < 0) || (reg > 1023)) {
		LOG_ERROR("Invalid SP register %d requested!", reg);
		printf("Invalid SP register %d requested!\n", reg);
		return ERROR_FAIL;
	}

	/* Transfer value to special purpose register */
	opcode = 0x7fe003a6 | ((reg & 0x1f) << 16) | ((reg & 0x3e0) << 6); /* mtspr reg, r31 */
	return mpc57xx_exec_inst(jtag_info, opcode, val, &val, MPC57XX_EI_VAL);
}

static int mpc57xx_read_cr(struct mpc5xxx_jtag *jtag_info, uint32_t *val)
{
	int retval;
	uint32_t opcode, r31;

	/* Preserve R31 */
	retval = mpc57xx_read_gpr(jtag_info, MPC5XXX_REG_R31, &r31);
	if (retval)
		return retval;

	/* Transfer condition register to R31 */
	opcode = 0x7fe00026; /* mfcr r31 */
	retval = mpc57xx_exec_inst(jtag_info, opcode, 0, val, 0);

	/* Restore R31 */
	return mpc57xx_write_gpr(jtag_info, MPC5XXX_REG_R31, r31);
}

static int mpc57xx_write_cr(struct mpc5xxx_jtag *jtag_info, uint32_t val)
{
	uint32_t opcode;

	/* Transfer value to condition register */
	opcode = 0x7feff120; /* mtcr r31 */
	return mpc57xx_exec_inst(jtag_info, opcode, val, &val, MPC57XX_EI_VAL);
}

static int mpc57xx_read_ctr(struct mpc5xxx_jtag *jtag_info, uint32_t *val)
{
	int retval;
	uint32_t opcode, r31;

	/* Preserve R31 - may not be necessary ? */
	retval = mpc57xx_read_gpr(jtag_info, MPC5XXX_REG_R31, &r31);
	if (retval)
		return retval;

	/* Transfer condition register to R31 */
	opcode = 0x7fe902a6; /* mfctr r31 */
	retval = mpc57xx_exec_inst(jtag_info, opcode, 0, val, 0);

	/* Restore R31 */
	return mpc57xx_write_gpr(jtag_info, MPC5XXX_REG_R31, r31);
}

static int mpc57xx_write_ctr(struct mpc5xxx_jtag *jtag_info, uint32_t val)
{
	uint32_t opcode;

	/* Transfer value to condition register */
	opcode = 0x7fe903a6; /* mtctr r31 */
	return mpc57xx_exec_inst(jtag_info, opcode, val, &val, MPC57XX_EI_VAL);
}

static int mpc57xx_read_lr(struct mpc5xxx_jtag *jtag_info, uint32_t *val)
{
	int retval;
	uint32_t opcode, r31;

	/* Preserve R31 - may not be necessary ? */
	retval = mpc57xx_read_gpr(jtag_info, MPC5XXX_REG_R31, &r31);
	if (retval)
		return retval;

	/* Transfer condition register to R31 */
	opcode = 0x7fe802a6; /* mflr r31 */
	retval = mpc57xx_exec_inst(jtag_info, opcode, 0, val, 0);

	/* Restore R31 */
	return mpc57xx_write_gpr(jtag_info, MPC5XXX_REG_R31, r31);
}

static int mpc57xx_write_lr(struct mpc5xxx_jtag *jtag_info, uint32_t val)
{
	uint32_t opcode;

	/* Transfer value to condition register */
	opcode = 0x7fe803a6; /* mtlr r31 */
	return mpc57xx_exec_inst(jtag_info, opcode, val, &val, MPC57XX_EI_VAL);
}

int mpc57xx_read_reg(struct mpc5xxx_jtag *jtag_info, int reg,
		uint32_t *val)
{
	int retval;
	struct mpc5xxx_cpuscr scr;

	if (reg < 0) {
		LOG_ERROR("Invalid negative register %d requested", reg);
		printf("Invalid GP register %d requested!\n", reg);
		*val = 0;
		return ERROR_OK;
	} else if (reg < 32) {
		retval = mpc57xx_read_gpr(jtag_info, reg, val);
	} else if (reg == MPC5XXX_REG_PC) {
		retval = mpc5xxx_once_cpuscr_read(jtag_info, &scr);
		*val = scr.pc;
		printf("Read of PC = 0x%08x\n", *val);
	} else if (reg == MPC5XXX_REG_MSR) {
		retval = mpc5xxx_once_cpuscr_read(jtag_info, &scr);
		*val = scr.msr;
	} else if (reg == MPC5XXX_REG_CND) {
		retval = mpc57xx_read_cr(jtag_info, val);
	} else if (reg == MPC5XXX_REG_LR) {
		retval = mpc57xx_read_lr(jtag_info, val);
	} else if (reg == MPC5XXX_REG_CNT) { /* This is CTR ? */
		retval = mpc57xx_read_ctr(jtag_info, val);
	} else if (reg == MPC5XXX_REG_XER) {
		retval = mpc57xx_read_spr(jtag_info, 1, val);
	} else if (reg == MPC5XXX_REG_MQ) {
		/* Ignore MQ */
		*val = 0;
		retval = ERROR_OK;
	} else {
		/* Faking read of register */
		*val = 0;
		retval = ERROR_OK;
	}

	return retval;
}

int mpc57xx_write_reg(struct mpc5xxx_jtag *jtag_info, int reg,
		uint32_t val)
{
	int retval;
	struct mpc5xxx_cpuscr scr;

	if (reg < 0) {
		LOG_ERROR("Invalid negative register %d requested", reg);
		printf("Invalid GP register %d requested!\n", reg);
		return ERROR_OK;
	} else if (reg < 32) {
		retval = mpc57xx_write_gpr(jtag_info, reg, val);
	} else if (reg == MPC5XXX_REG_PC) {
		retval = mpc5xxx_once_cpuscr_read(jtag_info, &scr);
		if (retval)
			return retval;
		printf("Writing PC = 0x%08x\n", val);
		scr.pc = val;
		retval = mpc5xxx_once_cpuscr_write(jtag_info, &scr);
	} else if (reg == MPC5XXX_REG_MSR) {
		retval = mpc5xxx_once_cpuscr_read(jtag_info, &scr);
		if (retval)
			return retval;
		scr.msr = val;
		retval = mpc5xxx_once_cpuscr_write(jtag_info, &scr);
	} else if (reg == MPC5XXX_REG_CND) {
		retval = mpc57xx_write_cr(jtag_info, val);
	} else if (reg == MPC5XXX_REG_LR) {
		retval = mpc57xx_write_lr(jtag_info, val);
	} else if (reg == MPC5XXX_REG_CNT) { /* This is CTR ? */
		retval = mpc57xx_write_ctr(jtag_info, val);
	} else if (reg == MPC5XXX_REG_XER) {
		retval = mpc57xx_write_spr(jtag_info, 1, val);
	} else if (reg == MPC5XXX_REG_MQ) {
		/* Ignore MQ */
		retval = ERROR_OK;
	} else {
		/*printf("Faking write_reg %d\n", reg);*/
		retval = ERROR_OK;
	}

	return retval;
}

int mpc57xx_jtag_read_regs(struct mpc5xxx_jtag *jtag_info, uint32_t *regs, uint32_t * saved_ctl)
{
	int i, retval;

	/* read core registers */
	/* read in PC and MSR first, as they get wiped out and affected by later ops */
	struct mpc5xxx_cpuscr scr;

	retval = mpc5xxx_once_cpuscr_read(jtag_info, &scr);
	if (retval)
			return retval;

	regs[MPC5XXX_REG_PC]  = scr.pc ;
	regs[MPC5XXX_REG_MSR] = scr.msr ;
	if (saved_ctl)
		*saved_ctl = scr.ctl ;


	/* turn off interrupts and lie about it in regs[msr] */

	scr.msr &= ~MPC5XXX_MSR_EE ;
	mpc5xxx_once_cpuscr_write(jtag_info, &scr) ;

	for (i = 0; i < MPC5XXX_NUMCOREREGS - 1; i++) {
		if (!(i == MPC5XXX_REG_PC || i == MPC5XXX_REG_MSR)) {
			retval = mpc57xx_read_reg(jtag_info, i, regs + i);
			if (retval)
				return retval;
			if (i == (MPC5XXX_NUMCOREREGS >> 1))
				keep_alive();
		}
	}

	return ERROR_OK;
}

int mpc57xx_jtag_write_regs(struct mpc5xxx_jtag *jtag_info, uint32_t *regs)
{
	int i, retval;

	/*
	 * And now the rest of registers
	 */
	for (i = 0; i < MPC5XXX_NUMCOREREGS - 1; i++) {
		retval = mpc57xx_write_reg(jtag_info, i, regs[i]);
		if (retval)
			return retval;
	}

	return ERROR_OK;
}
