/***************************************************************************
 *   Copyright (C) 2009 by Mathias Kuester                                 *
 *   mkdorg@users.sourceforge.net                                          *
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

#ifndef OPENOCD_TARGET_DSP563XX_ONCE_H
#define OPENOCD_TARGET_DSP563XX_ONCE_H

#include <jtag/jtag.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define DSP563XX_ONCE_OCR_EX	(1<<5)
#define DSP563XX_ONCE_OCR_GO	(1<<6)
#define DSP563XX_ONCE_OCR_RW	(1<<7)

#define DSP563XX_ONCE_OSCR_OS1	(1<<7)
#define DSP563XX_ONCE_OSCR_OS0	(1<<6)
#define DSP563XX_ONCE_OSCR_HIT	(1<<5)
#define DSP563XX_ONCE_OSCR_TO	(1<<4)
#define DSP563XX_ONCE_OSCR_MBO	(1<<3)
#define DSP563XX_ONCE_OSCR_SWO	(1<<2)
#define DSP563XX_ONCE_OSCR_IME	(1<<1)
#define DSP563XX_ONCE_OSCR_TME	(1<<0)

#define DSP563XX_ONCE_OSCR_NORMAL_M	(0)
#define DSP563XX_ONCE_OSCR_STOPWAIT_M	(DSP563XX_ONCE_OSCR_OS0)
#define DSP563XX_ONCE_OSCR_BUSY_M	(DSP563XX_ONCE_OSCR_OS1)
#define DSP563XX_ONCE_OSCR_DEBUG_M	(DSP563XX_ONCE_OSCR_OS0|DSP563XX_ONCE_OSCR_OS1)

#define DSP563XX_ONCE_OSCR	0x000	/* status/ctrl reg. */
#define DSP563XX_ONCE_OMBC	0x001	/* memory breakp. reg. */
#define DSP563XX_ONCE_OBCR	0x002	/* breakp. ctrl reg */
#define DSP563XX_ONCE_OMLR0	0x005	/* memory limit reg */
#define DSP563XX_ONCE_OMLR1	0x006	/* memory limit reg */
#define DSP563XX_ONCE_OGDBR	0x009	/* gdb reg */
#define DSP563XX_ONCE_OPDBR	0x00A	/* pdb reg */
#define DSP563XX_ONCE_OPILR	0x00B	/* pil reg */
#define DSP563XX_ONCE_PDBGOTO	0x00C	/* pdb to go reg */
#define DSP563XX_ONCE_OTC	0x00D	/* trace cnt */
#define DSP563XX_ONCE_TAGB	0x00E	/* tags buffer */
#define DSP563XX_ONCE_OPABFR	0x00F	/* pab fetch reg */
#define DSP563XX_ONCE_OPABDR	0x010	/* pab decode reg */
#define DSP563XX_ONCE_OPABEX	0x011	/* pab exec reg */
#define DSP563XX_ONCE_OPABF11	0x012	/* trace buffer/inc ptr */
#define DSP563XX_ONCE_NOREG	0x01F	/* no register selected */

struct once_reg {
	const uint8_t num;
	const uint8_t addr;
	const uint8_t len;
	const char *name;
	uint32_t reg;
};

/** */
int dsp563xx_once_request_debug(struct jtag_tap *tap, int reset_state);
/** */
int dsp563xx_once_target_status(struct jtag_tap *tap);

/** once read registers */
int dsp563xx_once_read_register(struct jtag_tap *tap, int flush, struct once_reg *regs, int len);
/** once read register */
int dsp563xx_once_reg_read_ex(struct jtag_tap *tap, int flush, uint8_t reg, uint8_t len, uint32_t *data);
/** once read register */
int dsp563xx_once_reg_read(struct jtag_tap *tap, int flush, uint8_t reg, uint32_t *data);
/** once write register */
int dsp563xx_once_reg_write(struct jtag_tap *tap, int flush, uint8_t reg, uint32_t data);
/** single word instruction */
int dsp563xx_once_execute_sw_ir(struct jtag_tap *tap, int flush, uint32_t opcode);
/** double word instruction */
int dsp563xx_once_execute_dw_ir(struct jtag_tap *tap, int flush, uint32_t opcode, uint32_t operand);

#endif /* OPENOCD_TARGET_DSP563XX_ONCE_H */
