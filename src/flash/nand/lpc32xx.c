/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2011 Bjarne Steinsbo <bsteinsbo@gmail.com>              *
 *   Copyright (C) 2010 richard vegh <vegh.ricsi@gmail.com>                *
 *   Copyright (C) 2010 Oyvind Harboe <oyvind.harboe@zylin.com>            *
 *                                                                         *
 *   Based on a combination of the lpc3180 driver and code from            *
 *   uboot-2009.03-lpc32xx by Kevin Wells.                                 *
 *   Any bugs are mine. --BSt                                              *
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

#include "imp.h"
#include "lpc32xx.h"
#include <target/target.h>

static int lpc32xx_reset(struct nand_device *nand);
static int lpc32xx_controller_ready(struct nand_device *nand, int timeout);
static int lpc32xx_tc_ready(struct nand_device *nand, int timeout);
extern int nand_correct_data(struct nand_device *nand, u_char *dat,
		u_char *read_ecc, u_char *calc_ecc);

/* These are offset with the working area in IRAM when using DMA to
 * read/write data to the SLC controller.
 * - DMA descriptors will be put at start of working area,
 * - Hardware generated ECC will be stored at ECC_OFFS
 * - OOB will be read/written from/to SPARE_OFFS
 * - Actual page data will be read from/to DATA_OFFS
 * There are unused holes between the used areas.
 */
#define ECC_OFFS   0x120
#define SPARE_OFFS 0x140
#define DATA_OFFS  0x200

static const int sp_ooblayout[] = {
	10, 11, 12, 13, 14, 15
};
static const int lp_ooblayout[] = {
	40, 41, 42, 43, 44, 45,
	46, 47, 48, 49, 50, 51,
	52, 53, 54, 55, 56, 57,
	58, 59, 60, 61, 62, 63
};

struct dmac_ll {
	volatile uint32_t dma_src;
	volatile uint32_t dma_dest;
	volatile uint32_t next_lli;
	volatile uint32_t next_ctrl;
};

static struct dmac_ll dmalist[(2048/256) * 2 + 1];

/* nand device lpc32xx <target#> <oscillator_frequency>
 */
NAND_DEVICE_COMMAND_HANDLER(lpc32xx_nand_device_command)
{
	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t osc_freq;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], osc_freq);

	struct lpc32xx_nand_controller *lpc32xx_info;
	lpc32xx_info = malloc(sizeof(struct lpc32xx_nand_controller));
	nand->controller_priv = lpc32xx_info;

	lpc32xx_info->osc_freq = osc_freq;

	if ((lpc32xx_info->osc_freq < 1000) || (lpc32xx_info->osc_freq > 20000))
		LOG_WARNING("LPC32xx oscillator frequency should be between "
			"1000 and 20000 kHz, was %i",
			lpc32xx_info->osc_freq);

	lpc32xx_info->selected_controller = LPC32XX_NO_CONTROLLER;
	lpc32xx_info->sw_write_protection = 0;
	lpc32xx_info->sw_wp_lower_bound = 0x0;
	lpc32xx_info->sw_wp_upper_bound = 0x0;

	return ERROR_OK;
}

static int lpc32xx_pll(int fclkin, uint32_t pll_ctrl)
{
	int bypass = (pll_ctrl & 0x8000) >> 15;
	int direct = (pll_ctrl & 0x4000) >> 14;
	int feedback = (pll_ctrl & 0x2000) >> 13;
	int p = (1 << ((pll_ctrl & 0x1800) >> 11) * 2);
	int n = ((pll_ctrl & 0x0600) >> 9) + 1;
	int m = ((pll_ctrl & 0x01fe) >> 1) + 1;
	int lock = (pll_ctrl & 0x1);

	if (!lock)
		LOG_WARNING("PLL is not locked");

	if (!bypass && direct)	/* direct mode */
		return (m * fclkin) / n;

	if (bypass && !direct)	/* bypass mode */
		return fclkin / (2 * p);

	if (bypass & direct)	/* direct bypass mode */
		return fclkin;

	if (feedback)	/* integer mode */
		return m * (fclkin / n);
	else	/* non-integer mode */
		return (m / (2 * p)) * (fclkin / n);
}

static float lpc32xx_cycle_time(struct nand_device *nand)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t sysclk_ctrl, pwr_ctrl, hclkdiv_ctrl, hclkpll_ctrl;
	int sysclk;
	int hclk;
	int hclk_pll;
	float cycle;
	int retval;

	/* calculate timings */

	/* determine current SYSCLK (13'MHz or main oscillator) */
	retval = target_read_u32(target, 0x40004050, &sysclk_ctrl);
	if (retval != ERROR_OK) {
		LOG_ERROR("could not read SYSCLK_CTRL");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if ((sysclk_ctrl & 1) == 0)
		sysclk = lpc32xx_info->osc_freq;
	else
		sysclk = 13000;

	/* determine selected HCLK source */
	retval = target_read_u32(target, 0x40004044, &pwr_ctrl);
	if (retval != ERROR_OK) {
		LOG_ERROR("could not read HCLK_CTRL");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if ((pwr_ctrl & (1 << 2)) == 0)		/* DIRECT RUN mode */
		hclk = sysclk;
	else {
		retval = target_read_u32(target, 0x40004058, &hclkpll_ctrl);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not read HCLKPLL_CTRL");
			return ERROR_NAND_OPERATION_FAILED;
		}
		hclk_pll = lpc32xx_pll(sysclk, hclkpll_ctrl);

		retval = target_read_u32(target, 0x40004040, &hclkdiv_ctrl);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not read CLKDIV_CTRL");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (pwr_ctrl & (1 << 10))	/* ARM_CLK and HCLK use PERIPH_CLK */
			hclk = hclk_pll / (((hclkdiv_ctrl & 0x7c) >> 2) + 1);
		else	/* HCLK uses HCLK_PLL */
			hclk = hclk_pll / (1 << (hclkdiv_ctrl & 0x3));
	}

	LOG_DEBUG("LPC32xx HCLK currently clocked at %i kHz", hclk);

	cycle = (1.0 / hclk) * 1000000.0;

	return cycle;
}

static int lpc32xx_init(struct nand_device *nand)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int bus_width = nand->bus_width ? nand->bus_width : 8;
	int address_cycles = nand->address_cycles ? nand->address_cycles : 3;
	int page_size = nand->page_size ? nand->page_size : 512;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC32xx "
			"NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	/* sanitize arguments */
	if (bus_width != 8) {
		LOG_ERROR("LPC32xx doesn't support %i", bus_width);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}

	/* inform calling code about selected bus width */
	nand->bus_width = bus_width;

	if ((address_cycles < 3) || (address_cycles > 5)) {
		LOG_ERROR("LPC32xx driver doesn't support %i address cycles", address_cycles);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}

	if ((page_size != 512) && (page_size != 2048)) {
		LOG_ERROR("LPC32xx doesn't support page size %i", page_size);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}

	/* select MLC controller if none is currently selected */
	if (lpc32xx_info->selected_controller == LPC32XX_NO_CONTROLLER) {
		LOG_DEBUG("no LPC32xx NAND flash controller selected, "
			"using default 'slc'");
		lpc32xx_info->selected_controller = LPC32XX_SLC_CONTROLLER;
	}

	if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
		uint32_t mlc_icr_value = 0x0;
		float cycle;
		int twp, twh, trp, treh, trhz, trbwb, tcea;

		/* FLASHCLK_CTRL = 0x22 (enable clk for MLC) */
		retval = target_write_u32(target, 0x400040c8, 0x22);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set FLASHCLK_CTRL");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_CEH = 0x0 (Force nCE assert) */
		retval = target_write_u32(target, 0x200b804c, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_CEH");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_LOCK = 0xa25e (unlock protected registers) */
		retval = target_write_u32(target, 0x200b8044, 0xa25e);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_LOCK");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_ICR = configuration */
		if (lpc32xx_info->sw_write_protection)
			mlc_icr_value |= 0x8;
		if (page_size == 2048)
			mlc_icr_value |= 0x4;
		if (address_cycles == 4)
			mlc_icr_value |= 0x2;
		if (bus_width == 16)
			mlc_icr_value |= 0x1;
		retval = target_write_u32(target, 0x200b8030, mlc_icr_value);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ICR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* calculate NAND controller timings */
		cycle = lpc32xx_cycle_time(nand);

		twp = ((40 / cycle) + 1);
		twh = ((20 / cycle) + 1);
		trp = ((30 / cycle) + 1);
		treh = ((15 / cycle) + 1);
		trhz = ((30 / cycle) + 1);
		trbwb = ((100 / cycle) + 1);
		tcea = ((45 / cycle) + 1);

		/* MLC_LOCK = 0xa25e (unlock protected registers) */
		retval = target_write_u32(target, 0x200b8044, 0xa25e);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_LOCK");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_TIME_REG */
		retval = target_write_u32(target, 0x200b8034,
				(twp & 0xf)
				| ((twh & 0xf) << 4)
				| ((trp & 0xf) << 8)
				| ((treh & 0xf) << 12)
				| ((trhz & 0x7) << 16)
				| ((trbwb & 0x1f) << 19)
				| ((tcea & 0x3) << 24));
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_TIME_REG");
			return ERROR_NAND_OPERATION_FAILED;
		}

		retval = lpc32xx_reset(nand);
		if (retval != ERROR_OK)
			return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
		float cycle;
		int r_setup, r_hold, r_width, r_rdy;
		int w_setup, w_hold, w_width, w_rdy;

		/* FLASHCLK_CTRL = 0x05 (enable clk for SLC) */
		retval = target_write_u32(target, 0x400040c8, 0x05);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set FLASHCLK_CTRL");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* after reset set other registers of SLC,
		 * so reset calling is here at the beginning
		 */
		retval = lpc32xx_reset(nand);
		if (retval != ERROR_OK)
			return ERROR_NAND_OPERATION_FAILED;

		/* SLC_CFG =
			Force nCE assert,
			DMA ECC enabled,
			ECC enabled,
			DMA burst enabled,
			DMA read from SLC,
			WIDTH = bus_width)
		*/
		retval = target_write_u32(target, 0x20020014,
				0x3e | ((bus_width == 16) ? 1 : 0));
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set SLC_CFG");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* SLC_IEN = 3 (INT_RDY_EN = 1) ,(INT_TC_STAT = 1) */
		retval = target_write_u32(target, 0x20020020, 0x03);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set SLC_IEN");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* DMA configuration */

		/* DMACLK_CTRL = 0x01 (enable clock for DMA controller) */
		retval = target_write_u32(target, 0x400040e8, 0x01);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set DMACLK_CTRL");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* DMACConfig = DMA enabled*/
		retval = target_write_u32(target, 0x31000030, 0x01);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set DMACConfig");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* calculate NAND controller timings */
		cycle = lpc32xx_cycle_time(nand);

		r_setup = w_setup = 0;
		r_hold = w_hold = 10 / cycle;
		r_width = 30 / cycle;
		w_width = 40 / cycle;
		r_rdy = w_rdy = 100 / cycle;

		/* SLC_TAC: SLC timing arcs register */
		retval = target_write_u32(target, 0x2002002c,
				(r_setup & 0xf)
				| ((r_hold & 0xf) << 4)
				| ((r_width & 0xf) << 8)
				| ((r_rdy & 0xf) << 12)
				| ((w_setup & 0xf) << 16)
				| ((w_hold & 0xf) << 20)
				| ((w_width & 0xf) << 24)
				| ((w_rdy & 0xf) << 28));
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set SLC_TAC");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int lpc32xx_reset(struct nand_device *nand)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use "
			"LPC32xx NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc32xx_info->selected_controller == LPC32XX_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC32xx NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
		/* MLC_CMD = 0xff (reset controller and NAND device) */
		retval = target_write_u32(target, 0x200b8000, 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_CMD");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (!lpc32xx_controller_ready(nand, 100)) {
			LOG_ERROR("LPC32xx MLC NAND controller timed out "
				"after reset");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}
	} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
		/* SLC_CTRL = 0x6 (ECC_CLEAR, SW_RESET) */
		retval = target_write_u32(target, 0x20020010, 0x6);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set SLC_CTRL");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (!lpc32xx_controller_ready(nand, 100)) {
			LOG_ERROR("LPC32xx SLC NAND controller timed out "
				"after reset");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}
	}

	return ERROR_OK;
}

static int lpc32xx_command(struct nand_device *nand, uint8_t command)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use "
			"LPC32xx NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc32xx_info->selected_controller == LPC32XX_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC32xx NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
		/* MLC_CMD = command */
		retval = target_write_u32(target, 0x200b8000, command);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_CMD");
			return ERROR_NAND_OPERATION_FAILED;
		}
	} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
		/* SLC_CMD = command */
		retval = target_write_u32(target, 0x20020008, command);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set SLC_CMD");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int lpc32xx_address(struct nand_device *nand, uint8_t address)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use "
			"LPC32xx NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc32xx_info->selected_controller == LPC32XX_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC32xx NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
		/* MLC_ADDR = address */
		retval = target_write_u32(target, 0x200b8004, address);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
	} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
		/* SLC_ADDR = address */
		retval = target_write_u32(target, 0x20020004, address);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set SLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int lpc32xx_write_data(struct nand_device *nand, uint16_t data)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use "
			"LPC32xx NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc32xx_info->selected_controller == LPC32XX_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC32xx NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
		/* MLC_DATA = data */
		retval = target_write_u32(target, 0x200b0000, data);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_DATA");
			return ERROR_NAND_OPERATION_FAILED;
		}
	} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
		/* SLC_DATA = data */
		retval = target_write_u32(target, 0x20020000, data);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set SLC_DATA");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int lpc32xx_read_data(struct nand_device *nand, void *data)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC32xx "
			"NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc32xx_info->selected_controller == LPC32XX_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC32xx NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
		/* data = MLC_DATA, use sized access */
		if (nand->bus_width == 8) {
			uint8_t *data8 = data;
			retval = target_read_u8(target, 0x200b0000, data8);
		} else {
			LOG_ERROR("BUG: bus_width neither 8 nor 16 bit");
			return ERROR_NAND_OPERATION_FAILED;
		}
		if (retval != ERROR_OK) {
			LOG_ERROR("could not read MLC_DATA");
			return ERROR_NAND_OPERATION_FAILED;
		}
	} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
		uint32_t data32;

		/* data = SLC_DATA, must use 32-bit access */
		retval = target_read_u32(target, 0x20020000, &data32);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not read SLC_DATA");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (nand->bus_width == 8) {
			uint8_t *data8 = data;
			*data8 = data32 & 0xff;
		} else {
			LOG_ERROR("BUG: bus_width neither 8 nor 16 bit");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int lpc32xx_write_page_mlc(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size,
	uint8_t *oob, uint32_t oob_size)
{
	struct target *target = nand->target;
	int retval;
	uint8_t status;
	static uint8_t page_buffer[512];
	static uint8_t oob_buffer[6];
	int quarter, num_quarters;

	/* MLC_CMD = sequential input */
	retval = target_write_u32(target, 0x200b8000, NAND_CMD_SEQIN);
	if (retval != ERROR_OK) {
		LOG_ERROR("could not set MLC_CMD");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (nand->page_size == 512) {
		/* MLC_ADDR = 0x0 (one column cycle) */
		retval = target_write_u32(target, 0x200b8004, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_ADDR = row */
		retval = target_write_u32(target, 0x200b8004, page & 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
		retval = target_write_u32(target, 0x200b8004,
				(page >> 8) & 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (nand->address_cycles == 4) {
			retval = target_write_u32(target, 0x200b8004,
					(page >> 16) & 0xff);
			if (retval != ERROR_OK) {
				LOG_ERROR("could not set MLC_ADDR");
				return ERROR_NAND_OPERATION_FAILED;
			}
		}
	} else {
		/* MLC_ADDR = 0x0 (two column cycles) */
		retval = target_write_u32(target, 0x200b8004, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
		retval = target_write_u32(target, 0x200b8004, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_ADDR = row */
		retval = target_write_u32(target, 0x200b8004, page & 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
		retval = target_write_u32(target, 0x200b8004,
				(page >> 8) & 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	/* when using the MLC controller, we have to treat a large page device
	 * as being made out of four quarters, each the size of a small page
	 * device
	 */
	num_quarters = (nand->page_size == 2048) ? 4 : 1;

	for (quarter = 0; quarter < num_quarters; quarter++) {
		int thisrun_data_size = (data_size > 512) ? 512 : data_size;
		int thisrun_oob_size = (oob_size > 6) ? 6 : oob_size;

		memset(page_buffer, 0xff, 512);
		if (data) {
			memcpy(page_buffer, data, thisrun_data_size);
			data_size -= thisrun_data_size;
			data += thisrun_data_size;
		}

		memset(oob_buffer, 0xff, 6);
		if (oob) {
			memcpy(oob_buffer, oob, thisrun_oob_size);
			oob_size -= thisrun_oob_size;
			oob += thisrun_oob_size;
		}

		/* write MLC_ECC_ENC_REG to start encode cycle */
		retval = target_write_u32(target, 0x200b8008, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ECC_ENC_REG");
			return ERROR_NAND_OPERATION_FAILED;
		}

		retval = target_write_memory(target, 0x200a8000,
				4, 128, page_buffer);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_BUF (data)");
			return ERROR_NAND_OPERATION_FAILED;
		}
		retval = target_write_memory(target, 0x200a8000,
				1, 6, oob_buffer);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_BUF (oob)");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* write MLC_ECC_AUTO_ENC_REG to start auto encode */
		retval = target_write_u32(target, 0x200b8010, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ECC_AUTO_ENC_REG");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (!lpc32xx_controller_ready(nand, 1000)) {
			LOG_ERROR("timeout while waiting for "
				"completion of auto encode cycle");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	/* MLC_CMD = auto program command */
	retval = target_write_u32(target, 0x200b8000, NAND_CMD_PAGEPROG);
	if (retval != ERROR_OK) {
		LOG_ERROR("could not set MLC_CMD");
		return ERROR_NAND_OPERATION_FAILED;
	}

	retval = nand_read_status(nand, &status);
	if (retval != ERROR_OK) {
		LOG_ERROR("couldn't read status");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (status & NAND_STATUS_FAIL) {
		LOG_ERROR("write operation didn't pass, status: 0x%2.2x",
			status);
		return ERROR_NAND_OPERATION_FAILED;
	}

	return ERROR_OK;
}

/* SLC controller in !raw mode will use target cpu to read/write nand from/to
 * target internal memory.  The transfer to/from flash is done by DMA.  This
 * function sets up the dma linked list in host memory for later transfer to
 * target.
 */
static int lpc32xx_make_dma_list(uint32_t target_mem_base, uint32_t page_size,
	int do_read)
{
	uint32_t i, dmasrc, ctrl, ecc_ctrl, oob_ctrl, dmadst;

	/* DMACCxControl =
		TransferSize =64,
		Source burst size =16,
		Destination burst size = 16,
		Source transfer width = 32 bit,
		Destination transfer width = 32 bit,
		Source AHB master select = M0,
		Destination AHB master select = M0,
		Source increment = 0, // set later
		Destination increment = 0, // set later
		Terminal count interrupt enable bit = 0 // set on last
	*/			/*
	 * Write Operation Sequence for Small Block NAND
	 * ----------------------------------------------------------
	 * 1. X'fer 256 bytes of data from Memory to Flash.
	 * 2. Copy generated ECC data from Register to Spare Area
	 * 3. X'fer next 256 bytes of data from Memory to Flash.
	 * 4. Copy generated ECC data from Register to Spare Area.
	 * 5. X'fer 16 bytes of Spare area from Memory to Flash.
	 * Read Operation Sequence for Small Block NAND
	 * ----------------------------------------------------------
	 * 1. X'fer 256 bytes of data from Flash to Memory.
	 * 2. Copy generated ECC data from Register to ECC calc Buffer.
	 * 3. X'fer next 256 bytes of data from Flash to Memory.
	 * 4. Copy generated ECC data from Register to ECC calc Buffer.
	 * 5. X'fer 16 bytes of Spare area from Flash to Memory.
	 * Write Operation Sequence for Large Block NAND
	 * ----------------------------------------------------------
	 * 1. Steps(1-4) of Write Operations repeated for four times
	 * which generates 16 DMA descriptors to X'fer 2048 bytes of
	 * data & 32 bytes of ECC data.
	 * 2. X'fer 64 bytes of Spare area from Memory to Flash.
	 * Read Operation Sequence for Large Block NAND
	 * ----------------------------------------------------------
	 * 1. Steps(1-4) of Read Operations repeated for four times
	 * which generates 16 DMA descriptors to X'fer 2048 bytes of
	 * data & 32 bytes of ECC data.
	 * 2. X'fer 64 bytes of Spare area from Flash to Memory.
	 */

	ctrl = (0x40 | 3 << 12 | 3 << 15 | 2 << 18 | 2 << 21 | 0 << 24
		| 0 << 25 | 0 << 26 | 0 << 27 | 0 << 31);

	/* DMACCxControl =
		TransferSize =1,
		Source burst size =4,
		Destination burst size = 4,
		Source transfer width = 32 bit,
		Destination transfer width = 32 bit,
		Source AHB master select = M0,
		Destination AHB master select = M0,
		Source increment = 0,
		Destination increment = 1,
		Terminal count interrupt enable bit = 0
	 */
	ecc_ctrl = 0x01 | 1 << 12 | 1 << 15 | 2 << 18 | 2 << 21 | 0 << 24
		| 0 << 25 | 0 << 26 | 1 << 27 | 0 << 31;

	/* DMACCxControl =
		TransferSize =16 for lp or 4 for sp,
		Source burst size =16,
		Destination burst size = 16,
		Source transfer width = 32 bit,
		Destination transfer width = 32 bit,
		Source AHB master select = M0,
		Destination AHB master select = M0,
		Source increment = 0, // set later
		Destination increment = 0, // set later
		Terminal count interrupt enable bit = 1 // set on last
	 */
	oob_ctrl = (page_size == 2048 ? 0x10 : 0x04)
		| 3 << 12 | 3 << 15 | 2 << 18 | 2 << 21 | 0 << 24
		| 0 << 25 | 0 << 26 | 0 << 27 | 1 << 31;
	if (do_read) {
		ctrl |= 1 << 27;/* Destination increment = 1 */
		oob_ctrl |= 1 << 27;	/* Destination increment = 1 */
		dmasrc = 0x20020038;	/* SLC_DMA_DATA */
		dmadst = target_mem_base + DATA_OFFS;
	} else {
		ctrl |= 1 << 26;/* Source increment = 1 */
		oob_ctrl |= 1 << 26;	/* Source increment = 1 */
		dmasrc = target_mem_base + DATA_OFFS;
		dmadst = 0x20020038;	/* SLC_DMA_DATA */
	}
	/*
	 * Write Operation Sequence for Small Block NAND
	 * ----------------------------------------------------------
	 * 1. X'fer 256 bytes of data from Memory to Flash.
	 * 2. Copy generated ECC data from Register to Spare Area
	 * 3. X'fer next 256 bytes of data from Memory to Flash.
	 * 4. Copy generated ECC data from Register to Spare Area.
	 * 5. X'fer 16 bytes of Spare area from Memory to Flash.
	 * Read Operation Sequence for Small Block NAND
	 * ----------------------------------------------------------
	 * 1. X'fer 256 bytes of data from Flash to Memory.
	 * 2. Copy generated ECC data from Register to ECC calc Buffer.
	 * 3. X'fer next 256 bytes of data from Flash to Memory.
	 * 4. Copy generated ECC data from Register to ECC calc Buffer.
	 * 5. X'fer 16 bytes of Spare area from Flash to Memory.
	 * Write Operation Sequence for Large Block NAND
	 * ----------------------------------------------------------
	 * 1. Steps(1-4) of Write Operations repeated for four times
	 * which generates 16 DMA descriptors to X'fer 2048 bytes of
	 * data & 32 bytes of ECC data.
	 * 2. X'fer 64 bytes of Spare area from Memory to Flash.
	 * Read Operation Sequence for Large Block NAND
	 * ----------------------------------------------------------
	 * 1. Steps(1-4) of Read Operations repeated for four times
	 * which generates 16 DMA descriptors to X'fer 2048 bytes of
	 * data & 32 bytes of ECC data.
	 * 2. X'fer 64 bytes of Spare area from Flash to Memory.
	 */
	for (i = 0; i < page_size/0x100; i++) {
		dmalist[i*2].dma_src = (do_read ? dmasrc : (dmasrc + i * 256));
		dmalist[i*2].dma_dest = (do_read ? (dmadst + i * 256) : dmadst);
		dmalist[i*2].next_lli =
			target_mem_base + (i*2 + 1) * sizeof(struct dmac_ll);
		dmalist[i*2].next_ctrl = ctrl;

		dmalist[(i*2) + 1].dma_src = 0x20020034;/* SLC_ECC */
		dmalist[(i*2) + 1].dma_dest =
			target_mem_base + ECC_OFFS + i * 4;
		dmalist[(i*2) + 1].next_lli =
			target_mem_base + (i*2 + 2) * sizeof(struct dmac_ll);
		dmalist[(i*2) + 1].next_ctrl = ecc_ctrl;

	}
	if (do_read)
		dmadst = target_mem_base + SPARE_OFFS;
	else {
		dmasrc = target_mem_base + SPARE_OFFS;
		dmalist[(i*2) - 1].next_lli = 0;/* last link = null on write */
		dmalist[(i*2) - 1].next_ctrl |= (1 << 31);	/* Set TC enable */
	}
	dmalist[i*2].dma_src = dmasrc;
	dmalist[i*2].dma_dest = dmadst;
	dmalist[i*2].next_lli = 0;
	dmalist[i*2].next_ctrl = oob_ctrl;

	return i * 2 + 1;	/* Number of descriptors */
}

static int lpc32xx_start_slc_dma(struct nand_device *nand, uint32_t count,
	int do_wait)
{
	struct target *target = nand->target;
	int retval;

	/* DMACIntTCClear = ch0 */
	retval = target_write_u32(target, 0x31000008, 1);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not set DMACIntTCClear");
		return retval;
	}

	/* DMACIntErrClear = ch0 */
	retval = target_write_u32(target, 0x31000010, 1);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not set DMACIntErrClear");
		return retval;
	}

	/* DMACCxConfig=
		E=1,
		SrcPeripheral = 1 (SLC),
		DestPeripheral = 1 (SLC),
		FlowCntrl = 2 (Pher -> Mem, DMA),
		IE = 0,
		ITC = 0,
		L= 0,
		H=0
	*/
	retval = target_write_u32(target, 0x31000110,
			1 | 1<<1 | 1<<6 | 2<<11 | 0<<14
			| 0<<15 | 0<<16 | 0<<18);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not set DMACC0Config");
		return retval;
	}

	/* SLC_CTRL = 3 (START DMA), ECC_CLEAR */
	retval = target_write_u32(target, 0x20020010, 0x3);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not set SLC_CTRL");
		return retval;
	}

	/* SLC_ICR = 2, INT_TC_CLR, clear pending TC*/
	retval = target_write_u32(target, 0x20020028, 2);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not set SLC_ICR");
		return retval;
	}

	/* SLC_TC */
	retval = target_write_u32(target, 0x20020030, count);
	if (retval != ERROR_OK) {
		LOG_ERROR("lpc32xx_start_slc_dma: Could not set SLC_TC");
		return retval;
	}

	/* Wait finish */
	if (do_wait && !lpc32xx_tc_ready(nand, 100)) {
		LOG_ERROR("timeout while waiting for completion of DMA");
		return ERROR_NAND_OPERATION_FAILED;
	}

	return retval;
}

static int lpc32xx_dma_ready(struct nand_device *nand, int timeout)
{
	struct target *target = nand->target;

	LOG_DEBUG("lpc32xx_dma_ready count start=%d", timeout);

	do {
		uint32_t tc_stat;
		uint32_t err_stat;
		int retval;

		/* Read DMACRawIntTCStat */
		retval = target_read_u32(target, 0x31000014, &tc_stat);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DMACRawIntTCStat");
			return 0;
		}
		/* Read DMACRawIntErrStat */
		retval = target_read_u32(target, 0x31000018, &err_stat);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DMACRawIntErrStat");
			return 0;
		}
		if ((tc_stat | err_stat) & 1) {
			LOG_DEBUG("lpc32xx_dma_ready count=%d",
				timeout);
			if (err_stat & 1) {
				LOG_ERROR("lpc32xx_dma_ready "
					"DMA error, aborted");
				return 0;
			} else
				return 1;
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

static uint32_t slc_ecc_copy_to_buffer(uint8_t *spare,
	const uint32_t *ecc, int count)
{
	int i;
	for (i = 0; i < (count * 3); i += 3) {
		uint32_t ce = ecc[i/3];
		ce = ~(ce << 2) & 0xFFFFFF;
		spare[i+2] = (uint8_t)(ce & 0xFF); ce >>= 8;
		spare[i+1] = (uint8_t)(ce & 0xFF); ce >>= 8;
		spare[i]   = (uint8_t)(ce & 0xFF);
	}
	return 0;
}

static void lpc32xx_dump_oob(uint8_t *oob, uint32_t oob_size)
{
	int addr = 0;
	while (oob_size > 0) {
		LOG_DEBUG("%02x: %02x %02x %02x %02x %02x %02x %02x %02x", addr,
			oob[0], oob[1], oob[2], oob[3],
			oob[4], oob[5], oob[6], oob[7]);
		oob += 8;
		addr += 8;
		oob_size -= 8;
	}
}

static int lpc32xx_write_page_slc(struct nand_device *nand,
	struct working_area *pworking_area,
	uint32_t page, uint8_t *data,
	uint32_t data_size, uint8_t *oob,
	uint32_t oob_size)
{
	struct target *target = nand->target;
	int retval;
	uint32_t target_mem_base;

	LOG_DEBUG("SLC write page %" PRIx32 " data=%d, oob=%d, "
		"data_size=%" PRIu32 ", oob_size=%" PRIu32,
		page, data != 0, oob != 0, data_size, oob_size);

	target_mem_base = pworking_area->address;
	/*
	 * Skip writing page which has all 0xFF data as this will
	 * generate 0x0 value.
	 */
	if (data && !oob) {
		uint32_t i, all_ff = 1;
		for (i = 0; i < data_size; i++)
			if (data[i] != 0xFF) {
				all_ff = 0;
				break;
			}
		if (all_ff)
			return ERROR_OK;
	}
	/* Make the dma descriptors in local memory */
	int nll = lpc32xx_make_dma_list(target_mem_base, nand->page_size, 0);
	/* Write them to target.
	   XXX: Assumes host and target have same byte sex.
	*/
	retval = target_write_memory(target, target_mem_base, 4,
			nll * sizeof(struct dmac_ll) / 4,
			(uint8_t *)dmalist);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not write DMA descriptors to IRAM");
		return retval;
	}

	retval = nand_page_command(nand, page, NAND_CMD_SEQIN, !data);
	if (retval != ERROR_OK) {
		LOG_ERROR("NAND_CMD_SEQIN failed");
		return retval;
	}

	/* SLC_CFG =
	       Force nCE assert,
	       DMA ECC enabled,
	       ECC enabled,
	       DMA burst enabled,
	       DMA write to SLC,
	       WIDTH = bus_width
	*/
	retval = target_write_u32(target, 0x20020014, 0x3c);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not set SLC_CFG");
		return retval;
	}
	if (data) {
		/* Write data to target */
		static uint8_t fdata[2048];
		memset(fdata, 0xFF, nand->page_size);
		memcpy(fdata, data, data_size);
		retval = target_write_memory(target,
				target_mem_base + DATA_OFFS,
				4, nand->page_size/4, fdata);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not write data to IRAM");
			return retval;
		}

		/* Write first descriptor to DMA controller */
		retval = target_write_memory(target, 0x31000100, 4,
				sizeof(struct dmac_ll) / 4,
				(uint8_t *)dmalist);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not write DMA descriptor to DMAC");
			return retval;
		}

		/* Start xfer of data from iram to flash using DMA */
		int tot_size = nand->page_size;
		tot_size += tot_size == 2048 ? 64 : 16;
		retval = lpc32xx_start_slc_dma(nand, tot_size, 0);
		if (retval != ERROR_OK) {
			LOG_ERROR("DMA failed");
			return retval;
		}

		/* Wait for DMA to finish.  SLC is not finished at this stage */
		if (!lpc32xx_dma_ready(nand, 100)) {
			LOG_ERROR("Data DMA failed during write");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}	/* data xfer */

	/* Copy OOB to iram */
	static uint8_t foob[64];
	int foob_size = nand->page_size == 2048 ? 64 : 16;
	memset(foob, 0xFF, foob_size);
	if (oob)	/* Raw mode */
		memcpy(foob, oob, oob_size);
	else {
		/* Get HW generated ECC, made while writing data */
		int ecc_count = nand->page_size == 2048 ? 8 : 2;
		static uint32_t hw_ecc[8];
		retval = target_read_memory(target, target_mem_base + ECC_OFFS,
				4, ecc_count, (uint8_t *)hw_ecc);
		if (retval != ERROR_OK) {
			LOG_ERROR("Reading hw generated ECC from IRAM failed");
			return retval;
		}
		/* Copy to oob, at correct offsets */
		static uint8_t ecc[24];
		slc_ecc_copy_to_buffer(ecc, hw_ecc, ecc_count);
		const int *layout = nand->page_size == 2048 ? lp_ooblayout : sp_ooblayout;
		int i;
		for (i = 0; i < ecc_count * 3; i++)
			foob[layout[i]] = ecc[i];
		lpc32xx_dump_oob(foob, foob_size);
	}
	retval = target_write_memory(target, target_mem_base + SPARE_OFFS, 4,
			foob_size / 4, foob);
	if (retval != ERROR_OK) {
		LOG_ERROR("Writing OOB to IRAM failed");
		return retval;
	}

	/* Write OOB descriptor to DMA controller */
	retval = target_write_memory(target, 0x31000100, 4,
			sizeof(struct dmac_ll) / 4,
			(uint8_t *)(&dmalist[nll-1]));
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not write OOB DMA descriptor to DMAC");
		return retval;
	}
	if (data) {
		/* Only restart DMA with last descriptor,
		 * don't setup SLC again */

		/* DMACIntTCClear = ch0 */
		retval = target_write_u32(target, 0x31000008, 1);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not set DMACIntTCClear");
			return retval;
		}
		/* DMACCxConfig=
		 * E=1,
		 * SrcPeripheral = 1 (SLC),
		 * DestPeripheral = 1 (SLC),
		 * FlowCntrl = 2 (Pher -> Mem, DMA),
		 * IE = 0,
		 * ITC = 0,
		 * L= 0,
		 * H=0
		*/
		retval = target_write_u32(target, 0x31000110,
				1 | 1<<1 | 1<<6 | 2<<11 | 0<<14
				| 0<<15 | 0<<16 | 0<<18);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not set DMACC0Config");
			return retval;
		}
		/* Wait finish */
		if (!lpc32xx_tc_ready(nand, 100)) {
			LOG_ERROR("timeout while waiting for "
				"completion of DMA");
			return ERROR_NAND_OPERATION_FAILED;
		}
	} else {
		/* Start xfer of data from iram to flash using DMA */
		retval = lpc32xx_start_slc_dma(nand, foob_size, 1);
		if (retval != ERROR_OK) {
			LOG_ERROR("DMA OOB failed");
			return retval;
		}
	}

	/* Let NAND start actual writing */
	retval = nand_write_finish(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("nand_write_finish failed");
		return retval;
	}

	return ERROR_OK;
}

static int lpc32xx_write_page(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size,
	uint8_t *oob, uint32_t oob_size)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC32xx "
			"NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc32xx_info->selected_controller == LPC32XX_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC32xx NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
		if (!data && oob) {
			LOG_ERROR("LPC32xx MLC controller can't write "
				"OOB data only");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		if (oob && (oob_size > 24)) {
			LOG_ERROR("LPC32xx MLC controller can't write more "
				"than 6 bytes for each quarter's OOB data");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		if (data_size > (uint32_t)nand->page_size) {
			LOG_ERROR("data size exceeds page size");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		retval = lpc32xx_write_page_mlc(nand, page, data, data_size,
				oob, oob_size);
	} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
		struct working_area *pworking_area;
		if (!data && oob) {
			/*
			 * if oob only mode is active original method is used
			 * as SLC controller hangs during DMA interworking. (?)
			 * Anyway the code supports the oob only mode below.
			 */
			return nand_write_page_raw(nand, page, data,
				data_size, oob, oob_size);
		}
		retval = target_alloc_working_area(target,
				nand->page_size + DATA_OFFS,
				&pworking_area);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't allocate working area in "
				"LPC internal RAM");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		retval = lpc32xx_write_page_slc(nand, pworking_area, page,
				data, data_size, oob, oob_size);
		target_free_working_area(target, pworking_area);
	}

	return retval;
}

static int lpc32xx_read_page_mlc(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size,
	uint8_t *oob, uint32_t oob_size)
{
	struct target *target = nand->target;
	static uint8_t page_buffer[2048];
	static uint8_t oob_buffer[64];
	uint32_t page_bytes_done = 0;
	uint32_t oob_bytes_done = 0;
	uint32_t mlc_isr;
	int retval;

	if (!data && oob) {
		/* MLC_CMD = Read OOB
		 * we can use the READOOB command on both small and large page
		 * devices, as the controller translates the 0x50 command to
		 * a 0x0 with appropriate positioning of the serial buffer
		 * read pointer
		 */
		retval = target_write_u32(target, 0x200b8000, NAND_CMD_READOOB);
	} else {
		/* MLC_CMD = Read0 */
		retval = target_write_u32(target, 0x200b8000, NAND_CMD_READ0);
	}
	if (retval != ERROR_OK) {
		LOG_ERROR("could not set MLC_CMD");
		return ERROR_NAND_OPERATION_FAILED;
	}
	if (nand->page_size == 512) {
		/* small page device
		 * MLC_ADDR = 0x0 (one column cycle) */
		retval = target_write_u32(target, 0x200b8004, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_ADDR = row */
		retval = target_write_u32(target, 0x200b8004, page & 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
		retval = target_write_u32(target, 0x200b8004,
				(page >> 8) & 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (nand->address_cycles == 4) {
			retval = target_write_u32(target, 0x200b8004,
					(page >> 16) & 0xff);
			if (retval != ERROR_OK) {
				LOG_ERROR("could not set MLC_ADDR");
				return ERROR_NAND_OPERATION_FAILED;
			}
		}
	} else {
		/* large page device
		 * MLC_ADDR = 0x0 (two column cycles) */
		retval = target_write_u32(target, 0x200b8004, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
		retval = target_write_u32(target, 0x200b8004, 0x0);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_ADDR = row */
		retval = target_write_u32(target, 0x200b8004, page & 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}
		retval = target_write_u32(target, 0x200b8004,
				(page >> 8) & 0xff);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ADDR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		/* MLC_CMD = Read Start */
		retval = target_write_u32(target, 0x200b8000,
				NAND_CMD_READSTART);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_CMD");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	while (page_bytes_done < (uint32_t)nand->page_size) {
		/* MLC_ECC_AUTO_DEC_REG = dummy */
		retval = target_write_u32(target, 0x200b8014, 0xaa55aa55);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not set MLC_ECC_AUTO_DEC_REG");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (!lpc32xx_controller_ready(nand, 1000)) {
			LOG_ERROR("timeout while waiting for "
				"completion of auto decode cycle");
			return ERROR_NAND_OPERATION_FAILED;
		}

		retval = target_read_u32(target, 0x200b8048, &mlc_isr);
		if (retval != ERROR_OK) {
			LOG_ERROR("could not read MLC_ISR");
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (mlc_isr & 0x8) {
			if (mlc_isr & 0x40) {
				LOG_ERROR("uncorrectable error detected: "
					"0x%2.2x", (unsigned)mlc_isr);
				return ERROR_NAND_OPERATION_FAILED;
			}

			LOG_WARNING("%i symbol error detected and corrected",
				((int)(((mlc_isr & 0x30) >> 4) + 1)));
		}

		if (data) {
			retval = target_read_memory(target, 0x200a8000, 4, 128,
					page_buffer + page_bytes_done);
			if (retval != ERROR_OK) {
				LOG_ERROR("could not read MLC_BUF (data)");
				return ERROR_NAND_OPERATION_FAILED;
			}
		}

		if (oob) {
			retval = target_read_memory(target, 0x200a8000, 4, 4,
					oob_buffer + oob_bytes_done);
			if (retval != ERROR_OK) {
				LOG_ERROR("could not read MLC_BUF (oob)");
				return ERROR_NAND_OPERATION_FAILED;
			}
		}

		page_bytes_done += 512;
		oob_bytes_done += 16;
	}

	if (data)
		memcpy(data, page_buffer, data_size);

	if (oob)
		memcpy(oob, oob_buffer, oob_size);

	return ERROR_OK;
}

static int lpc32xx_read_page_slc(struct nand_device *nand,
	struct working_area *pworking_area,
	uint32_t page, uint8_t *data,
	uint32_t data_size, uint8_t *oob,
	uint32_t oob_size)
{
	struct target *target = nand->target;
	int retval;
	uint32_t target_mem_base;

	LOG_DEBUG("SLC read page %" PRIx32 " data=%" PRIu32 ", oob=%" PRIu32,
		page, data_size, oob_size);

	target_mem_base = pworking_area->address;

	/* Make the dma descriptors in local memory */
	int nll = lpc32xx_make_dma_list(target_mem_base, nand->page_size, 1);
	/* Write them to target.
	   XXX: Assumes host and target have same byte sex.
	*/
	retval = target_write_memory(target, target_mem_base, 4,
			nll * sizeof(struct dmac_ll) / 4,
			(uint8_t *)dmalist);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not write DMA descriptors to IRAM");
		return retval;
	}

	retval = nand_page_command(nand, page, NAND_CMD_READ0, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("lpc32xx_read_page_slc: NAND_CMD_READ0 failed");
		return retval;
	}

	/* SLC_CFG =
	       Force nCE assert,
	       DMA ECC enabled,
	       ECC enabled,
	       DMA burst enabled,
	       DMA read from SLC,
	       WIDTH = bus_width
	*/
	retval = target_write_u32(target, 0x20020014, 0x3e);
	if (retval != ERROR_OK) {
		LOG_ERROR("lpc32xx_read_page_slc: Could not set SLC_CFG");
		return retval;
	}

	/* Write first descriptor to DMA controller */
	retval = target_write_memory(target, 0x31000100, 4,
			sizeof(struct dmac_ll) / 4, (uint8_t *)dmalist);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not write DMA descriptor to DMAC");
		return retval;
	}

	/* Start xfer of data from flash to iram using DMA */
	int tot_size = nand->page_size;
	tot_size += nand->page_size == 2048 ? 64 : 16;
	retval = lpc32xx_start_slc_dma(nand, tot_size, 1);
	if (retval != ERROR_OK) {
		LOG_ERROR("lpc32xx_read_page_slc: DMA read failed");
		return retval;
	}

	/* Copy data from iram */
	if (data) {
		retval = target_read_memory(target, target_mem_base + DATA_OFFS,
				4, data_size/4, data);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read data from IRAM");
			return retval;
		}
	}
	if (oob) {
		/* No error correction, just return data as read from flash */
		retval = target_read_memory(target,
				target_mem_base + SPARE_OFFS, 4,
				oob_size/4, oob);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read OOB from IRAM");
			return retval;
		}
		return ERROR_OK;
	}

	/* Copy OOB from flash, stored in IRAM */
	static uint8_t foob[64];
	retval = target_read_memory(target, target_mem_base + SPARE_OFFS,
			4, nand->page_size == 2048 ? 16 : 4, foob);
	lpc32xx_dump_oob(foob, nand->page_size == 2048 ? 64 : 16);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read OOB from IRAM");
		return retval;
	}
	/* Copy ECC from HW, generated while reading */
	int ecc_count = nand->page_size == 2048 ? 8 : 2;
	static uint32_t hw_ecc[8];	/* max size */
	retval = target_read_memory(target, target_mem_base + ECC_OFFS, 4,
			ecc_count, (uint8_t *)hw_ecc);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read hw generated ECC from IRAM");
		return retval;
	}
	static uint8_t ecc[24];
	slc_ecc_copy_to_buffer(ecc, hw_ecc, ecc_count);
	/* Copy ECC from flash using correct layout */
	static uint8_t fecc[24];/* max size */
	const int *layout = nand->page_size == 2048 ? lp_ooblayout : sp_ooblayout;
	int i;
	for (i = 0; i < ecc_count * 3; i++)
		fecc[i] = foob[layout[i]];
	/* Compare ECC and possibly correct data */
	for (i = 0; i < ecc_count; i++) {
		retval = nand_correct_data(nand, data + 256*i, &fecc[i * 3],
				&ecc[i * 3]);
		if (retval > 0)
			LOG_WARNING("error detected and corrected: %" PRIu32 "/%d",
				page, i);
		if (retval < 0)
			break;
	}
	if (i == ecc_count)
		retval = ERROR_OK;
	else {
		LOG_ERROR("uncorrectable error detected: %" PRIu32 "/%d", page, i);
		retval = ERROR_NAND_OPERATION_FAILED;
	}
	return retval;
}

static int lpc32xx_read_page(struct nand_device *nand, uint32_t page,
	uint8_t *data, uint32_t data_size,
	uint8_t *oob, uint32_t oob_size)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC32xx "
			"NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc32xx_info->selected_controller == LPC32XX_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC32xx NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
		if (data_size > (uint32_t)nand->page_size) {
			LOG_ERROR("data size exceeds page size");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}
		retval = lpc32xx_read_page_mlc(nand, page, data, data_size,
				oob, oob_size);
	} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
		struct working_area *pworking_area;

		retval = target_alloc_working_area(target,
				nand->page_size + 0x200,
				&pworking_area);
		if (retval != ERROR_OK) {
			LOG_ERROR("Can't allocate working area in "
				"LPC internal RAM");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		retval = lpc32xx_read_page_slc(nand, pworking_area, page,
				data, data_size, oob, oob_size);
		target_free_working_area(target, pworking_area);
	}

	return retval;
}

static int lpc32xx_controller_ready(struct nand_device *nand, int timeout)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC32xx "
			"NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	LOG_DEBUG("lpc32xx_controller_ready count start=%d", timeout);

	do {
		if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
			uint8_t status;

			/* Read MLC_ISR, wait for controller to become ready */
			retval = target_read_u8(target, 0x200b8048, &status);
			if (retval != ERROR_OK) {
				LOG_ERROR("could not set MLC_STAT");
				return ERROR_NAND_OPERATION_FAILED;
			}

			if (status & 2) {
				LOG_DEBUG("lpc32xx_controller_ready count=%d",
					timeout);
				return 1;
			}
		} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
			uint32_t status;

			/* Read SLC_STAT and check READY bit */
			retval = target_read_u32(target, 0x20020018, &status);
			if (retval != ERROR_OK) {
				LOG_ERROR("could not set SLC_STAT");
				return ERROR_NAND_OPERATION_FAILED;
			}

			if (status & 1) {
				LOG_DEBUG("lpc32xx_controller_ready count=%d",
					timeout);
				return 1;
			}
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

static int lpc32xx_nand_ready(struct nand_device *nand, int timeout)
{
	struct lpc32xx_nand_controller *lpc32xx_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC32xx "
			"NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	LOG_DEBUG("lpc32xx_nand_ready count start=%d", timeout);

	do {
		if (lpc32xx_info->selected_controller == LPC32XX_MLC_CONTROLLER) {
			uint8_t status = 0x0;

			/* Read MLC_ISR, wait for NAND flash device to
			 * become ready */
			retval = target_read_u8(target, 0x200b8048, &status);
			if (retval != ERROR_OK) {
				LOG_ERROR("could not read MLC_ISR");
				return ERROR_NAND_OPERATION_FAILED;
			}

			if (status & 1) {
				LOG_DEBUG("lpc32xx_nand_ready count end=%d",
					timeout);
				return 1;
			}
		} else if (lpc32xx_info->selected_controller == LPC32XX_SLC_CONTROLLER) {
			uint32_t status = 0x0;

			/* Read SLC_STAT and check READY bit */
			retval = target_read_u32(target, 0x20020018, &status);
			if (retval != ERROR_OK) {
				LOG_ERROR("could not read SLC_STAT");
				return ERROR_NAND_OPERATION_FAILED;
			}

			if (status & 1) {
				LOG_DEBUG("lpc32xx_nand_ready count end=%d",
					timeout);
				return 1;
			}
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

static int lpc32xx_tc_ready(struct nand_device *nand, int timeout)
{
	struct target *target = nand->target;

	LOG_DEBUG("lpc32xx_tc_ready count start=%d", timeout);

	do {
		uint32_t status = 0x0;
		int retval;
		/* Read SLC_INT_STAT and check INT_TC_STAT bit */
		retval = target_read_u32(target, 0x2002001c, &status);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read SLC_INT_STAT");
			return 0;
		}
		if (status & 2) {
			LOG_DEBUG("lpc32xx_tc_ready count=%d", timeout);
			return 1;
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

COMMAND_HANDLER(handle_lpc32xx_select_command)
{
	struct lpc32xx_nand_controller *lpc32xx_info = NULL;
	char *selected[] = {
		"no", "mlc", "slc"
	};

	if ((CMD_ARGC < 1) || (CMD_ARGC > 3))
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned num;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num);
	struct nand_device *nand = get_nand_device_by_num(num);
	if (!nand) {
		command_print(CMD, "nand device '#%s' is out of bounds",
			CMD_ARGV[0]);
		return ERROR_OK;
	}

	lpc32xx_info = nand->controller_priv;

	if (CMD_ARGC >= 2) {
		if (strcmp(CMD_ARGV[1], "mlc") == 0) {
			lpc32xx_info->selected_controller =
				LPC32XX_MLC_CONTROLLER;
		} else if (strcmp(CMD_ARGV[1], "slc") == 0) {
			lpc32xx_info->selected_controller =
				LPC32XX_SLC_CONTROLLER;
		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "%s controller selected",
		selected[lpc32xx_info->selected_controller]);

	return ERROR_OK;
}

static const struct command_registration lpc32xx_exec_command_handlers[] = {
	{
		.name = "select",
		.handler = handle_lpc32xx_select_command,
		.mode = COMMAND_EXEC,
		.help = "select MLC or SLC controller (default is MLC)",
		.usage = "bank_id ['mlc'|'slc' ]",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration lpc32xx_command_handler[] = {
	{
		.name = "lpc32xx",
		.mode = COMMAND_ANY,
		.help = "LPC32xx NAND flash controller commands",
		.usage = "",
		.chain = lpc32xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct nand_flash_controller lpc32xx_nand_controller = {
	.name = "lpc32xx",
	.commands = lpc32xx_command_handler,
	.nand_device_command = lpc32xx_nand_device_command,
	.init = lpc32xx_init,
	.reset = lpc32xx_reset,
	.command = lpc32xx_command,
	.address = lpc32xx_address,
	.write_data = lpc32xx_write_data,
	.read_data = lpc32xx_read_data,
	.write_page = lpc32xx_write_page,
	.read_page = lpc32xx_read_page,
	.nand_ready = lpc32xx_nand_ready,
};
