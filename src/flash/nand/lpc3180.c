/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *
 *   Copyright (C) 2010 richard vegh <vegh.ricsi@gmail.com>                *
 *   Copyright (C) 2010 Oyvind Harboe <oyvind.harboe@zylin.com>            *
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
#include "lpc3180.h"
#include <target/target.h>

static int lpc3180_reset(struct nand_device *nand);
static int lpc3180_controller_ready(struct nand_device *nand, int timeout);
static int lpc3180_tc_ready(struct nand_device *nand, int timeout);

#define ECC_OFFS   0x120
#define SPARE_OFFS 0x140
#define DATA_OFFS   0x200

/* nand device lpc3180 <target#> <oscillator_frequency>
 */
NAND_DEVICE_COMMAND_HANDLER(lpc3180_nand_device_command)
{
	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t osc_freq;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], osc_freq);

	struct lpc3180_nand_controller *lpc3180_info;
	lpc3180_info = malloc(sizeof(struct lpc3180_nand_controller));
	nand->controller_priv = lpc3180_info;

	lpc3180_info->osc_freq = osc_freq;

	if ((lpc3180_info->osc_freq < 1000) || (lpc3180_info->osc_freq > 20000))
		LOG_WARNING(
			"LPC3180 oscillator frequency should be between 1000 and 20000 kHz, was %i",
			lpc3180_info->osc_freq);
	lpc3180_info->selected_controller = LPC3180_NO_CONTROLLER;
	lpc3180_info->sw_write_protection = 0;
	lpc3180_info->sw_wp_lower_bound = 0x0;
	lpc3180_info->sw_wp_upper_bound = 0x0;

	return ERROR_OK;
}

static int lpc3180_pll(int fclkin, uint32_t pll_ctrl)
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

	if (feedback)			/* integer mode */
		return m * (fclkin / n);
	else					/* non-integer mode */
		return (m / (2 * p)) * (fclkin / n);
}

static float lpc3180_cycle_time(struct nand_device *nand)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t sysclk_ctrl, pwr_ctrl, hclkdiv_ctrl, hclkpll_ctrl;
	int sysclk;
	int hclk;
	int hclk_pll;
	float cycle;

	/* calculate timings */

	/* determine current SYSCLK (13'MHz or main oscillator) */
	target_read_u32(target, 0x40004050, &sysclk_ctrl);

	if ((sysclk_ctrl & 1) == 0)
		sysclk = lpc3180_info->osc_freq;
	else
		sysclk = 13000;

	/* determine selected HCLK source */
	target_read_u32(target, 0x40004044, &pwr_ctrl);

	if ((pwr_ctrl & (1 << 2)) == 0)	/* DIRECT RUN mode */
		hclk = sysclk;
	else {
		target_read_u32(target, 0x40004058, &hclkpll_ctrl);
		hclk_pll = lpc3180_pll(sysclk, hclkpll_ctrl);

		target_read_u32(target, 0x40004040, &hclkdiv_ctrl);

		if (pwr_ctrl & (1 << 10)) /* ARM_CLK and HCLK use PERIPH_CLK */
			hclk = hclk_pll / (((hclkdiv_ctrl & 0x7c) >> 2) + 1);
		else /* HCLK uses HCLK_PLL */
			hclk = hclk_pll / (1 << (hclkdiv_ctrl & 0x3));
	}

	LOG_DEBUG("LPC3180 HCLK currently clocked at %i kHz", hclk);

	cycle = (1.0 / hclk) * 1000000.0;

	return cycle;
}

static int lpc3180_init(struct nand_device *nand)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;
	int bus_width = nand->bus_width ? : 8;
	int address_cycles = nand->address_cycles ? : 3;
	int page_size = nand->page_size ? : 512;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	/* sanitize arguments */
	if ((bus_width != 8) && (bus_width != 16)) {
		LOG_ERROR("LPC3180 only supports 8 or 16 bit bus width, not %i", bus_width);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}

	/* The LPC3180 only brings out 8 bit NAND data bus, but the controller
	 * would support 16 bit, too, so we just warn about this for now
	 */
	if (bus_width == 16)
		LOG_WARNING("LPC3180 only supports 8 bit bus width");

	/* inform calling code about selected bus width */
	nand->bus_width = bus_width;

	if ((address_cycles != 3) && (address_cycles != 4)) {
		LOG_ERROR("LPC3180 only supports 3 or 4 address cycles, not %i", address_cycles);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}

	if ((page_size != 512) && (page_size != 2048)) {
		LOG_ERROR("LPC3180 only supports 512 or 2048 byte pages, not %i", page_size);
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;
	}

	/* select MLC controller if none is currently selected */
	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER) {
		LOG_DEBUG("no LPC3180 NAND flash controller selected, using default 'mlc'");
		lpc3180_info->selected_controller = LPC3180_MLC_CONTROLLER;
	}

	if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
		uint32_t mlc_icr_value = 0x0;
		float cycle;
		int twp, twh, trp, treh, trhz, trbwb, tcea;

		/* FLASHCLK_CTRL = 0x22 (enable clock for MLC flash controller) */
		target_write_u32(target, 0x400040c8, 0x22);

		/* MLC_CEH = 0x0 (Force nCE assert) */
		target_write_u32(target, 0x200b804c, 0x0);

		/* MLC_LOCK = 0xa25e (unlock protected registers) */
		target_write_u32(target, 0x200b8044, 0xa25e);

		/* MLC_ICR = configuration */
		if (lpc3180_info->sw_write_protection)
			mlc_icr_value |= 0x8;
		if (page_size == 2048)
			mlc_icr_value |= 0x4;
		if (address_cycles == 4)
			mlc_icr_value |= 0x2;
		if (bus_width == 16)
			mlc_icr_value |= 0x1;
		target_write_u32(target, 0x200b8030, mlc_icr_value);

		/* calculate NAND controller timings */
		cycle = lpc3180_cycle_time(nand);

		twp = ((40 / cycle) + 1);
		twh = ((20 / cycle) + 1);
		trp = ((30 / cycle) + 1);
		treh = ((15 / cycle) + 1);
		trhz = ((30 / cycle) + 1);
		trbwb = ((100 / cycle) + 1);
		tcea = ((45 / cycle) + 1);

		/* MLC_LOCK = 0xa25e (unlock protected registers) */
		target_write_u32(target, 0x200b8044, 0xa25e);

		/* MLC_TIME_REG */
		target_write_u32(target, 0x200b8034, (twp & 0xf) | ((twh & 0xf) << 4) |
			((trp & 0xf) << 8) | ((treh & 0xf) << 12) | ((trhz & 0x7) << 16) |
			((trbwb & 0x1f) << 19) | ((tcea & 0x3) << 24));

		lpc3180_reset(nand);
	} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
		float cycle;
		int r_setup, r_hold, r_width, r_rdy;
		int w_setup, w_hold, w_width, w_rdy;

		/* FLASHCLK_CTRL = 0x05 (enable clock for SLC flash controller) */
		target_write_u32(target, 0x400040c8, 0x05);

		/* after reset set other registers of SLC so reset calling is here at the begining*/
		lpc3180_reset(nand);

		/* SLC_CFG = 0x (Force nCE assert, DMA ECC enabled, ECC enabled, DMA burst enabled,
		 *DMA read from SLC, WIDTH = bus_width) */
		target_write_u32(target, 0x20020014, 0x3e | ((bus_width == 16) ? 1 : 0));

		/* SLC_IEN = 3 (INT_RDY_EN = 1) ,(INT_TC_STAT = 1) */
		target_write_u32(target, 0x20020020, 0x03);

		/* DMA configuration
		 * DMACLK_CTRL = 0x01 (enable clock for DMA controller) */
		target_write_u32(target, 0x400040e8, 0x01);
		/* DMACConfig = DMA enabled*/
		target_write_u32(target, 0x31000030, 0x01);


		/* calculate NAND controller timings */
		cycle = lpc3180_cycle_time(nand);

		r_setup = w_setup = 0;
		r_hold = w_hold = 10 / cycle;
		r_width = 30 / cycle;
		w_width = 40 / cycle;
		r_rdy = w_rdy = 100 / cycle;

		/* SLC_TAC: SLC timing arcs register */
		target_write_u32(target, 0x2002002c, (r_setup & 0xf) | ((r_hold & 0xf) << 4) |
			((r_width & 0xf) << 8) | ((r_rdy & 0xf) << 12) |  ((w_setup & 0xf) << 16) |
			((w_hold & 0xf) << 20) | ((w_width & 0xf) << 24) | ((w_rdy & 0xf) << 28));

	}

	return ERROR_OK;
}

static int lpc3180_reset(struct nand_device *nand)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
		/* MLC_CMD = 0xff (reset controller and NAND device) */
		target_write_u32(target, 0x200b8000, 0xff);

		if (!lpc3180_controller_ready(nand, 100)) {
			LOG_ERROR("LPC3180 NAND controller timed out after reset");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}
	} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
		/* SLC_CTRL = 0x6 (ECC_CLEAR, SW_RESET) */
		target_write_u32(target, 0x20020010, 0x6);

		if (!lpc3180_controller_ready(nand, 100)) {
			LOG_ERROR("LPC3180 NAND controller timed out after reset");
			return ERROR_NAND_OPERATION_TIMEOUT;
		}
	}

	return ERROR_OK;
}

static int lpc3180_command(struct nand_device *nand, uint8_t command)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
		/* MLC_CMD = command */
		target_write_u32(target, 0x200b8000, command);
	} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
		/* SLC_CMD = command */
		target_write_u32(target, 0x20020008, command);
	}

	return ERROR_OK;
}

static int lpc3180_address(struct nand_device *nand, uint8_t address)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
		/* MLC_ADDR = address */
		target_write_u32(target, 0x200b8004, address);
	} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
		/* SLC_ADDR = address */
		target_write_u32(target, 0x20020004, address);
	}

	return ERROR_OK;
}

static int lpc3180_write_data(struct nand_device *nand, uint16_t data)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
		/* MLC_DATA = data */
		target_write_u32(target, 0x200b0000, data);
	} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
		/* SLC_DATA = data */
		target_write_u32(target, 0x20020000, data);
	}

	return ERROR_OK;
}

static int lpc3180_read_data(struct nand_device *nand, void *data)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
		/* data = MLC_DATA, use sized access */
		if (nand->bus_width == 8) {
			uint8_t *data8 = data;
			target_read_u8(target, 0x200b0000, data8);
		} else if (nand->bus_width == 16) {
			uint16_t *data16 = data;
			target_read_u16(target, 0x200b0000, data16);
		} else {
			LOG_ERROR("BUG: bus_width neither 8 nor 16 bit");
			return ERROR_NAND_OPERATION_FAILED;
		}
	} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
		uint32_t data32;

		/* data = SLC_DATA, must use 32-bit access */
		target_read_u32(target, 0x20020000, &data32);

		if (nand->bus_width == 8) {
			uint8_t *data8 = data;
			*data8 = data32 & 0xff;
		} else if (nand->bus_width == 16) {
			uint16_t *data16 = data;
			*data16 = data32 & 0xffff;
		} else {
			LOG_ERROR("BUG: bus_width neither 8 nor 16 bit");
			return ERROR_NAND_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int lpc3180_write_page(struct nand_device *nand,
	uint32_t page,
	uint8_t *data,
	uint32_t data_size,
	uint8_t *oob,
	uint32_t oob_size)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;
	uint8_t status;
	uint8_t *page_buffer;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
		uint8_t *oob_buffer;
		int quarter, num_quarters;

		if (!data && oob) {
			LOG_ERROR("LPC3180 MLC controller can't write OOB data only");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		if (oob && (oob_size > 24)) {
			LOG_ERROR("LPC3180 MLC controller can't write more "
				"than 6 bytes for each quarter's OOB data");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		if (data_size > (uint32_t)nand->page_size) {
			LOG_ERROR("data size exceeds page size");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		/* MLC_CMD = sequential input */
		target_write_u32(target, 0x200b8000, NAND_CMD_SEQIN);

		page_buffer = malloc(512);
		oob_buffer = malloc(6);

		if (nand->page_size == 512) {
			/* MLC_ADDR = 0x0 (one column cycle) */
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);

			if (nand->address_cycles == 4)
				target_write_u32(target, 0x200b8004, (page >> 16) & 0xff);
		} else {
			/* MLC_ADDR = 0x0 (two column cycles) */
			target_write_u32(target, 0x200b8004, 0x0);
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);
		}

		/* when using the MLC controller, we have to treat a large page device
		 * as being made out of four quarters, each the size of a small page device
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
			target_write_u32(target, 0x200b8008, 0x0);

			target_write_memory(target, 0x200a8000,
				4, 128, page_buffer);
			target_write_memory(target, 0x200a8000,
				1, 6, oob_buffer);

			/* write MLC_ECC_AUTO_ENC_REG to start auto encode */
			target_write_u32(target, 0x200b8010, 0x0);

			if (!lpc3180_controller_ready(nand, 1000)) {
				LOG_ERROR("timeout while waiting for completion of auto encode cycle");
				free(page_buffer);
				free(oob_buffer);
				return ERROR_NAND_OPERATION_FAILED;
			}
		}

		/* MLC_CMD = auto program command */
		target_write_u32(target, 0x200b8000, NAND_CMD_PAGEPROG);

		retval = nand_read_status(nand, &status);
		if (retval != ERROR_OK) {
			LOG_ERROR("couldn't read status");
			free(page_buffer);
			free(oob_buffer);
			return ERROR_NAND_OPERATION_FAILED;
		}

		if (status & NAND_STATUS_FAIL) {
			LOG_ERROR("write operation didn't pass, status: 0x%2.2x", status);
			free(page_buffer);
			free(oob_buffer);
			return ERROR_NAND_OPERATION_FAILED;
		}

		free(page_buffer);
		free(oob_buffer);
	} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {

		/**********************************************************************
		*     Write both SLC NAND flash page main area and spare area.
		*     Small page -
		*      ------------------------------------------
		*     |    512 bytes main   |   16 bytes spare   |
		*      ------------------------------------------
		*     Large page -
		*      ------------------------------------------
		*     |   2048 bytes main   |   64 bytes spare   |
		*      ------------------------------------------
		*     If DMA & ECC enabled, then the ECC generated for the 1st 256-byte
		*     data is written to the 3rd word of the spare area. The ECC
		*     generated for the 2nd 256-byte data is written to the 4th word
		*     of the spare area. The ECC generated for the 3rd 256-byte data is
		*     written to the 7th word of the spare area. The ECC generated
		*     for the 4th 256-byte data is written to the 8th word of the
		*     spare area and so on.
		*
		**********************************************************************/

		int i = 0, target_mem_base;
		uint8_t *ecc_flash_buffer;
		struct working_area *pworking_area;

		if (lpc3180_info->is_bulk) {

			if (!data && oob) {
				/*if oob only mode is active original method is used as SLC
				 *controller hangs during DMA interworking. Anyway the code supports
				 *the oob only mode below. */
				return nand_write_page_raw(nand,
					page,
					data,
					data_size,
					oob,
					oob_size);
			}
			retval = nand_page_command(nand, page, NAND_CMD_SEQIN, !data);
			if (ERROR_OK != retval)
				return retval;

			/* allocate a working area */
			if (target->working_area_size < (uint32_t) nand->page_size + 0x200) {
				LOG_ERROR("Reserve at least 0x%x physical target working area",
					nand->page_size + 0x200);
				return ERROR_FLASH_OPERATION_FAILED;
			}
			if (target->working_area_phys%4) {
				LOG_ERROR(
					"Reserve the physical target working area at word boundary");
				return ERROR_FLASH_OPERATION_FAILED;
			}
			if (target_alloc_working_area(target, target->working_area_size,
				    &pworking_area) != ERROR_OK) {
				LOG_ERROR("no working area specified, can't read LPC internal flash");
				return ERROR_FLASH_OPERATION_FAILED;
			}
			target_mem_base = target->working_area_phys;

			if (nand->page_size == 2048)
				page_buffer = malloc(2048);
			else
				page_buffer = malloc(512);

			ecc_flash_buffer = malloc(64);

			/* SLC_CFG = 0x (Force nCE assert, DMA ECC enabled, ECC enabled, DMA burst
			 *enabled, DMA write to SLC, WIDTH = bus_width) */
			target_write_u32(target, 0x20020014, 0x3c);

			if (data && !oob) {
				/* set DMA LLI-s in target memory and in DMA*/
				for (i = 0; i < nand->page_size/0x100; i++) {

					int tmp;
					/* -------LLI for 256 byte block---------
					 * DMACC0SrcAddr = SRAM */
					target_write_u32(target,
						target_mem_base+0+i*32,
						target_mem_base+DATA_OFFS+i*256);
					if (i == 0)
						target_write_u32(target,
							0x31000100,
							target_mem_base+DATA_OFFS);
					/* DMACCxDestAddr = SLC_DMA_DATA */
					target_write_u32(target, target_mem_base+4+i*32, 0x20020038);
					if (i == 0)
						target_write_u32(target, 0x31000104, 0x20020038);
					/* DMACCxLLI = next element */
					tmp = (target_mem_base+(1+i*2)*16)&0xfffffffc;
					target_write_u32(target, target_mem_base+8+i*32, tmp);
					if (i == 0)
						target_write_u32(target, 0x31000108, tmp);
					/* DMACCxControl =  TransferSize =64, Source burst size =16,
					 * Destination burst size = 16, Source transfer width = 32 bit,
					 * Destination transfer width = 32 bit, Source AHB master select = M0,
					 * Destination AHB master select = M0, Source increment = 1,
					 * Destination increment = 0, Terminal count interrupt enable bit = 0*/
					target_write_u32(target,
						target_mem_base+12+i*32,
						0x40 | 3<<12 | 3<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 | 1<<26 |
						0<<27 | 0<<31);
					if (i == 0)
						target_write_u32(target,
							0x3100010c,
							0x40 | 3<<12 | 3<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 | 1<<26 |
							0<<27 | 0<<31);

					/* -------LLI for 3 byte ECC---------
					 * DMACC0SrcAddr = SLC_ECC*/
					target_write_u32(target, target_mem_base+16+i*32, 0x20020034);
					/* DMACCxDestAddr = SRAM */
					target_write_u32(target,
						target_mem_base+20+i*32,
						target_mem_base+SPARE_OFFS+8+16*(i>>1)+(i%2)*4);
					/* DMACCxLLI = next element */
					tmp = (target_mem_base+(2+i*2)*16)&0xfffffffc;
					target_write_u32(target, target_mem_base+24+i*32, tmp);
					/* DMACCxControl =  TransferSize =1, Source burst size =4,
					 * Destination burst size = 4, Source transfer width = 32 bit,
					 * Destination transfer width = 32 bit, Source AHB master select = M0,
					 * Destination AHB master select = M0, Source increment = 0,
					 * Destination increment = 1, Terminal count interrupt enable bit = 0*/
					target_write_u32(target,
						target_mem_base+28+i*32,
						0x01 | 1<<12 | 1<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 | 0<<26 | 1<<27 | 0<<
						31);
				}
			} else if (data && oob) {
				/* -------LLI for 512 or 2048 bytes page---------
				 * DMACC0SrcAddr = SRAM */
				target_write_u32(target, target_mem_base, target_mem_base+DATA_OFFS);
				target_write_u32(target, 0x31000100, target_mem_base+DATA_OFFS);
				/* DMACCxDestAddr = SLC_DMA_DATA */
				target_write_u32(target, target_mem_base+4, 0x20020038);
				target_write_u32(target, 0x31000104, 0x20020038);
				/* DMACCxLLI = next element */
				target_write_u32(target,
					target_mem_base+8,
					(target_mem_base+32)&0xfffffffc);
				target_write_u32(target, 0x31000108,
					(target_mem_base+32)&0xfffffffc);
				/* DMACCxControl =  TransferSize =512 or 128, Source burst size =16,
				 * Destination burst size = 16, Source transfer width = 32 bit,
				 * Destination transfer width = 32 bit, Source AHB master select = M0,
				 * Destination AHB master select = M0, Source increment = 1,
				 * Destination increment = 0, Terminal count interrupt enable bit = 0*/
				target_write_u32(target,
					target_mem_base+12,
					(nand->page_size ==
				 2048 ? 512 : 128) | 3<<12 | 3<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 |
				 1<<26 | 0<<27 | 0<<31);
				target_write_u32(target,
					0x3100010c,
					(nand->page_size ==
				 2048 ? 512 : 128) | 3<<12 | 3<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 |
				 1<<26 | 0<<27 | 0<<31);
				i = 1;
			} else if (!data && oob)
				i = 0;

			/* -------LLI for spare area---------
			 * DMACC0SrcAddr = SRAM*/
			target_write_u32(target, target_mem_base+0+i*32, target_mem_base+SPARE_OFFS);
			if (i == 0)
				target_write_u32(target, 0x31000100, target_mem_base+SPARE_OFFS);
			/* DMACCxDestAddr = SLC_DMA_DATA */
			target_write_u32(target, target_mem_base+4+i*32, 0x20020038);
			if (i == 0)
				target_write_u32(target, 0x31000104, 0x20020038);
			/* DMACCxLLI = next element = NULL */
			target_write_u32(target, target_mem_base+8+i*32, 0);
			if (i == 0)
				target_write_u32(target, 0x31000108, 0);
			/* DMACCxControl =  TransferSize =16 for large page or 4 for small page,
			 * Source burst size =16, Destination burst size = 16, Source transfer width = 32 bit,
			 * Destination transfer width = 32 bit, Source AHB master select = M0,
			 * Destination AHB master select = M0, Source increment = 1,
			 * Destination increment = 0, Terminal count interrupt enable bit = 0*/
			target_write_u32(target,
				target_mem_base+12+i*32,
				(nand->page_size ==
			 2048 ? 0x10 : 0x04) | 3<<12 | 3<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 | 1<<26 |
			 0<<27 | 0<<31);
			if (i == 0)
				target_write_u32(target, 0x3100010c,
					(nand->page_size == 2048 ?
					0x10 : 0x04) | 3<<12 | 3<<15 | 2<<18 | 2<<21 | 0<<24 |
					0<<25 | 1<<26 | 0<<27 | 0<<31);

			memset(ecc_flash_buffer, 0xff, 64);
			if (oob)
				memcpy(ecc_flash_buffer, oob, oob_size);
			target_write_memory(target,
				target_mem_base+SPARE_OFFS,
				4,
				16,
				ecc_flash_buffer);

			if (data) {
				memset(page_buffer, 0xff, nand->page_size == 2048 ? 2048 : 512);
				memcpy(page_buffer, data, data_size);
				target_write_memory(target,
					target_mem_base+DATA_OFFS,
					4,
					nand->page_size == 2048 ? 512 : 128,
					page_buffer);
			}

			free(page_buffer);
			free(ecc_flash_buffer);

			/* Enable DMA after channel set up !
			    LLI only works when DMA is the flow controller!
			*/
			/* DMACCxConfig= E=1, SrcPeripheral = 1 (SLC), DestPeripheral = 1 (SLC),
			 *FlowCntrl = 2 (Pher -> Mem, DMA), IE = 0, ITC = 0, L= 0, H=0*/
			target_write_u32(target,
				0x31000110,
				1 | 1<<1 | 1<<6 | 2<<11 | 0<<14 | 0<<15 | 0<<16 | 0<<18);

			/* SLC_CTRL = 3 (START DMA), ECC_CLEAR */
			target_write_u32(target, 0x20020010, 0x3);

			/* SLC_ICR = 2, INT_TC_CLR, clear pending TC*/
			target_write_u32(target, 0x20020028, 2);

			/* SLC_TC */
			if (!data && oob)
				target_write_u32(target, 0x20020030,
					(nand->page_size == 2048 ? 0x10 : 0x04));
			else
				target_write_u32(target, 0x20020030,
					(nand->page_size == 2048 ? 0x840 : 0x210));

			nand_write_finish(nand);

			if (!lpc3180_tc_ready(nand, 1000)) {
				LOG_ERROR("timeout while waiting for completion of DMA");
				return ERROR_NAND_OPERATION_FAILED;
			}

			target_free_working_area(target, pworking_area);

			LOG_INFO("Page =  0x%" PRIx32 " was written.", page);

		} else
			return nand_write_page_raw(nand, page, data, data_size, oob, oob_size);
	}

	return ERROR_OK;
}

static int lpc3180_read_page(struct nand_device *nand,
	uint32_t page,
	uint8_t *data,
	uint32_t data_size,
	uint8_t *oob,
	uint32_t oob_size)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;
	uint8_t *page_buffer;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (lpc3180_info->selected_controller == LPC3180_NO_CONTROLLER) {
		LOG_ERROR("BUG: no LPC3180 NAND flash controller selected");
		return ERROR_NAND_OPERATION_FAILED;
	} else if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
		uint8_t *oob_buffer;
		uint32_t page_bytes_done = 0;
		uint32_t oob_bytes_done = 0;
		uint32_t mlc_isr;

#if 0
		if (oob && (oob_size > 6)) {
			LOG_ERROR("LPC3180 MLC controller can't read more than 6 bytes of OOB data");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}
#endif

		if (data_size > (uint32_t)nand->page_size) {
			LOG_ERROR("data size exceeds page size");
			return ERROR_NAND_OPERATION_NOT_SUPPORTED;
		}

		if (nand->page_size == 2048) {
			page_buffer = malloc(2048);
			oob_buffer = malloc(64);
		} else {
			page_buffer = malloc(512);
			oob_buffer = malloc(16);
		}

		if (!data && oob) {
			/* MLC_CMD = Read OOB
			 * we can use the READOOB command on both small and large page devices,
			 * as the controller translates the 0x50 command to a 0x0 with appropriate
			 * positioning of the serial buffer read pointer
			 */
			target_write_u32(target, 0x200b8000, NAND_CMD_READOOB);
		} else {
			/* MLC_CMD = Read0 */
			target_write_u32(target, 0x200b8000, NAND_CMD_READ0);
		}

		if (nand->page_size == 512) {
			/* small page device
			 * MLC_ADDR = 0x0 (one column cycle) */
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);

			if (nand->address_cycles == 4)
				target_write_u32(target, 0x200b8004, (page >> 16) & 0xff);
		} else {
			/* large page device
			 * MLC_ADDR = 0x0 (two column cycles) */
			target_write_u32(target, 0x200b8004, 0x0);
			target_write_u32(target, 0x200b8004, 0x0);

			/* MLC_ADDR = row */
			target_write_u32(target, 0x200b8004, page & 0xff);
			target_write_u32(target, 0x200b8004, (page >> 8) & 0xff);

			/* MLC_CMD = Read Start */
			target_write_u32(target, 0x200b8000, NAND_CMD_READSTART);
		}

		while (page_bytes_done < (uint32_t)nand->page_size) {
			/* MLC_ECC_AUTO_DEC_REG = dummy */
			target_write_u32(target, 0x200b8014, 0xaa55aa55);

			if (!lpc3180_controller_ready(nand, 1000)) {
				LOG_ERROR("timeout while waiting for completion of auto decode cycle");
				free(page_buffer);
				free(oob_buffer);
				return ERROR_NAND_OPERATION_FAILED;
			}

			target_read_u32(target, 0x200b8048, &mlc_isr);

			if (mlc_isr & 0x8) {
				if (mlc_isr & 0x40) {
					LOG_ERROR("uncorrectable error detected: 0x%2.2x",
						(unsigned)mlc_isr);
					free(page_buffer);
					free(oob_buffer);
					return ERROR_NAND_OPERATION_FAILED;
				}

				LOG_WARNING("%i symbol error detected and corrected",
					((int)(((mlc_isr & 0x30) >> 4) + 1)));
			}

			if (data)
				target_read_memory(target,
					0x200a8000,
					4,
					128,
					page_buffer + page_bytes_done);

			if (oob)
				target_read_memory(target,
					0x200a8000,
					4,
					4,
					oob_buffer + oob_bytes_done);

			page_bytes_done += 512;
			oob_bytes_done += 16;
		}

		if (data)
			memcpy(data, page_buffer, data_size);

		if (oob)
			memcpy(oob, oob_buffer, oob_size);

		free(page_buffer);
		free(oob_buffer);
	} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {

		/**********************************************************************
		*     Read both SLC NAND flash page main area and spare area.
		*     Small page -
		*      ------------------------------------------
		*     |    512 bytes main   |   16 bytes spare   |
		*      ------------------------------------------
		*     Large page -
		*      ------------------------------------------
		*     |   2048 bytes main   |   64 bytes spare   |
		*      ------------------------------------------
		*     If DMA & ECC enabled, then the ECC generated for the 1st 256-byte
		*     data is compared with the 3rd word of the spare area. The ECC
		*     generated for the 2nd 256-byte data is compared with the 4th word
		*     of the spare area. The ECC generated for the 3rd 256-byte data is
		*     compared with the 7th word of the spare area. The ECC generated
		*     for the 4th 256-byte data is compared with the 8th word of the
		*     spare area and so on.
		*
		**********************************************************************/

		int retval, i, target_mem_base;
		uint8_t *ecc_hw_buffer;
		uint8_t *ecc_flash_buffer;
		struct working_area *pworking_area;

		if (lpc3180_info->is_bulk) {

			/* read always the data and also oob areas*/

			retval = nand_page_command(nand, page, NAND_CMD_READ0, 0);
			if (ERROR_OK != retval)
				return retval;

			/* allocate a working area */
			if (target->working_area_size < (uint32_t) nand->page_size + 0x200) {
				LOG_ERROR("Reserve at least 0x%x physical target working area",
					nand->page_size + 0x200);
				return ERROR_FLASH_OPERATION_FAILED;
			}
			if (target->working_area_phys%4) {
				LOG_ERROR(
					"Reserve the physical target working area at word boundary");
				return ERROR_FLASH_OPERATION_FAILED;
			}
			if (target_alloc_working_area(target, target->working_area_size,
				    &pworking_area) != ERROR_OK) {
				LOG_ERROR("no working area specified, can't read LPC internal flash");
				return ERROR_FLASH_OPERATION_FAILED;
			}
			target_mem_base = target->working_area_phys;

			if (nand->page_size == 2048)
				page_buffer = malloc(2048);
			else
				page_buffer = malloc(512);

			ecc_hw_buffer = malloc(32);
			ecc_flash_buffer = malloc(64);

			/* SLC_CFG = 0x (Force nCE assert, DMA ECC enabled, ECC enabled, DMA burst
			 *enabled, DMA read from SLC, WIDTH = bus_width) */
			target_write_u32(target, 0x20020014, 0x3e);

			/* set DMA LLI-s in target memory and in DMA*/
			for (i = 0; i < nand->page_size/0x100; i++) {
				int tmp;
				/* -------LLI for 256 byte block---------
				 * DMACC0SrcAddr = SLC_DMA_DATA*/
				target_write_u32(target, target_mem_base+0+i*32, 0x20020038);
				if (i == 0)
					target_write_u32(target, 0x31000100, 0x20020038);
				/* DMACCxDestAddr = SRAM */
				target_write_u32(target,
					target_mem_base+4+i*32,
					target_mem_base+DATA_OFFS+i*256);
				if (i == 0)
					target_write_u32(target,
						0x31000104,
						target_mem_base+DATA_OFFS);
				/* DMACCxLLI = next element */
				tmp = (target_mem_base+(1+i*2)*16)&0xfffffffc;
				target_write_u32(target, target_mem_base+8+i*32, tmp);
				if (i == 0)
					target_write_u32(target, 0x31000108, tmp);
				/* DMACCxControl =  TransferSize =64, Source burst size =16,
				 * Destination burst size = 16, Source transfer width = 32 bit,
				 * Destination transfer width = 32 bit, Source AHB master select = M0,
				 * Destination AHB master select = M0, Source increment = 0,
				 * Destination increment = 1, Terminal count interrupt enable bit = 0*/
				target_write_u32(target,
					target_mem_base+12+i*32,
					0x40 | 3<<12 | 3<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 | 0<<26 | 1<<27 | 0<<
					31);
				if (i == 0)
					target_write_u32(target,
						0x3100010c,
						0x40 | 3<<12 | 3<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 | 0<<26 | 1<<27 | 0<<
						31);

				/* -------LLI for 3 byte ECC---------
				 * DMACC0SrcAddr = SLC_ECC*/
				target_write_u32(target, target_mem_base+16+i*32, 0x20020034);
				/* DMACCxDestAddr = SRAM */
				target_write_u32(target,
					target_mem_base+20+i*32,
					target_mem_base+ECC_OFFS+i*4);
				/* DMACCxLLI = next element */
				tmp = (target_mem_base+(2+i*2)*16)&0xfffffffc;
				target_write_u32(target, target_mem_base+24+i*32, tmp);
				/* DMACCxControl =  TransferSize =1, Source burst size =4,
				 * Destination burst size = 4, Source transfer width = 32 bit,
				 * Destination transfer width = 32 bit, Source AHB master select = M0,
				 * Destination AHB master select = M0, Source increment = 0,
				 * Destination increment = 1, Terminal count interrupt enable bit = 0*/
				target_write_u32(target,
					target_mem_base+28+i*32,
					0x01 | 1<<12 | 1<<15 | 2<<18 | 2<<21 | 0<<24 | 0<<25 | 0<<26 | 1<<27 | 0<<
					31);
			}

			/* -------LLI for spare area---------
			 * DMACC0SrcAddr = SLC_DMA_DATA*/
			target_write_u32(target, target_mem_base+0+i*32, 0x20020038);
			/* DMACCxDestAddr = SRAM */
			target_write_u32(target, target_mem_base+4+i*32, target_mem_base+SPARE_OFFS);
			/* DMACCxLLI = next element = NULL */
			target_write_u32(target, target_mem_base+8+i*32, 0);
			/* DMACCxControl =  TransferSize =16 for large page or 4 for small page,
			 * Source burst size =16, Destination burst size = 16, Source transfer width = 32 bit,
			 * Destination transfer width = 32 bit, Source AHB master select = M0,
			 * Destination AHB master select = M0, Source increment = 0,
			 * Destination increment = 1, Terminal count interrupt enable bit = 0*/
			target_write_u32(target,
				target_mem_base + 12 + i * 32,
				(nand->page_size == 2048 ? 0x10 : 0x04) | 3<<12 | 3<<15 | 2<<18 | 2<<21 |
			 0<<24 | 0<<25 | 0<<26 | 1<<27 | 0<<31);

			/* Enable DMA after channel set up !
			    LLI only works when DMA is the flow controller!
			*/
			/* DMACCxConfig= E=1, SrcPeripheral = 1 (SLC), DestPeripheral = 1 (SLC),
			 *FlowCntrl = 2 (Pher-> Mem, DMA), IE = 0, ITC = 0, L= 0, H=0*/
			target_write_u32(target,
				0x31000110,
				1 | 1<<1 | 1<<6 |  2<<11 | 0<<14 | 0<<15 | 0<<16 | 0<<18);

			/* SLC_CTRL = 3 (START DMA), ECC_CLEAR */
			target_write_u32(target, 0x20020010, 0x3);

			/* SLC_ICR = 2, INT_TC_CLR, clear pending TC*/
			target_write_u32(target, 0x20020028, 2);

			/* SLC_TC */
			target_write_u32(target, 0x20020030,
				(nand->page_size == 2048 ? 0x840 : 0x210));

			if (!lpc3180_tc_ready(nand, 1000)) {
				LOG_ERROR("timeout while waiting for completion of DMA");
				free(page_buffer);
				free(ecc_hw_buffer);
				free(ecc_flash_buffer);
				target_free_working_area(target, pworking_area);
				return ERROR_NAND_OPERATION_FAILED;
			}

			if (data) {
				target_read_memory(target,
					target_mem_base+DATA_OFFS,
					4,
					nand->page_size == 2048 ? 512 : 128,
					page_buffer);
				memcpy(data, page_buffer, data_size);

				LOG_INFO("Page =  0x%" PRIx32 " was read.", page);

				/* check hw generated ECC for each 256 bytes block with the saved
				 *ECC in flash spare area*/
				int idx = nand->page_size/0x200;
				target_read_memory(target,
					target_mem_base+SPARE_OFFS,
					4,
					16,
					ecc_flash_buffer);
				target_read_memory(target,
					target_mem_base+ECC_OFFS,
					4,
					8,
					ecc_hw_buffer);
				for (i = 0; i < idx; i++) {
					if ((0x00ffffff & *(uint32_t *)(void *)(ecc_hw_buffer+i*8)) !=
							(0x00ffffff & *(uint32_t *)(void *)(ecc_flash_buffer+8+i*16)))
						LOG_WARNING(
							"ECC mismatch at 256 bytes size block= %d at page= 0x%" PRIx32,
							i * 2 + 1, page);
					if ((0x00ffffff & *(uint32_t *)(void *)(ecc_hw_buffer+4+i*8)) !=
							(0x00ffffff & *(uint32_t *)(void *)(ecc_flash_buffer+12+i*16)))
						LOG_WARNING(
							"ECC mismatch at 256 bytes size block= %d at page= 0x%" PRIx32,
							i * 2 + 2, page);
				}
			}

			if (oob)
				memcpy(oob, ecc_flash_buffer, oob_size);

			free(page_buffer);
			free(ecc_hw_buffer);
			free(ecc_flash_buffer);

			target_free_working_area(target, pworking_area);

		} else
			return nand_read_page_raw(nand, page, data, data_size, oob, oob_size);
	}

	return ERROR_OK;
}

static int lpc3180_controller_ready(struct nand_device *nand, int timeout)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	LOG_DEBUG("lpc3180_controller_ready count start=%d", timeout);

	do {
		if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
			uint8_t status;

			/* Read MLC_ISR, wait for controller to become ready */
			target_read_u8(target, 0x200b8048, &status);

			if (status & 2) {
				LOG_DEBUG("lpc3180_controller_ready count=%d",
					timeout);
				return 1;
			}
		} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
			uint32_t status;

			/* Read SLC_STAT and check READY bit */
			target_read_u32(target, 0x20020018, &status);

			if (status & 1) {
				LOG_DEBUG("lpc3180_controller_ready count=%d",
					timeout);
				return 1;
			}
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

static int lpc3180_nand_ready(struct nand_device *nand, int timeout)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	LOG_DEBUG("lpc3180_nand_ready count start=%d", timeout);

	do {
		if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER) {
			uint8_t status = 0x0;

			/* Read MLC_ISR, wait for NAND flash device to become ready */
			target_read_u8(target, 0x200b8048, &status);

			if (status & 1) {
				LOG_DEBUG("lpc3180_nand_ready count end=%d",
					timeout);
				return 1;
			}
		} else if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
			uint32_t status = 0x0;

			/* Read SLC_STAT and check READY bit */
			target_read_u32(target, 0x20020018, &status);

			if (status & 1) {
				LOG_DEBUG("lpc3180_nand_ready count end=%d",
					timeout);
				return 1;
			}
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

static int lpc3180_tc_ready(struct nand_device *nand, int timeout)
{
	struct lpc3180_nand_controller *lpc3180_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use LPC3180 NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	LOG_DEBUG("lpc3180_tc_ready count start=%d",
		timeout);

	do {
		if (lpc3180_info->selected_controller == LPC3180_SLC_CONTROLLER) {
			uint32_t status = 0x0;
			/* Read SLC_INT_STAT and check INT_TC_STAT bit */
			target_read_u32(target, 0x2002001c, &status);

			if (status & 2) {
				LOG_DEBUG("lpc3180_tc_ready count=%d",
					timeout);
				return 1;
			}
		}

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

COMMAND_HANDLER(handle_lpc3180_select_command)
{
	struct lpc3180_nand_controller *lpc3180_info = NULL;
	char *selected[] = {
		"no", "mlc", "slc"
	};

	if ((CMD_ARGC < 1) || (CMD_ARGC > 3))
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned num;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num);
	struct nand_device *nand = get_nand_device_by_num(num);
	if (!nand) {
		command_print(CMD, "nand device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	lpc3180_info = nand->controller_priv;

	if (CMD_ARGC >= 2) {
		if (strcmp(CMD_ARGV[1], "mlc") == 0)
			lpc3180_info->selected_controller = LPC3180_MLC_CONTROLLER;
		else if (strcmp(CMD_ARGV[1], "slc") == 0) {
			lpc3180_info->selected_controller = LPC3180_SLC_CONTROLLER;
			if (CMD_ARGC == 3 && strcmp(CMD_ARGV[2], "bulk") == 0)
				lpc3180_info->is_bulk = 1;
			else
				lpc3180_info->is_bulk = 0;
		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (lpc3180_info->selected_controller == LPC3180_MLC_CONTROLLER)
		command_print(CMD, "%s controller selected",
			selected[lpc3180_info->selected_controller]);
	else
		command_print(CMD,
			lpc3180_info->is_bulk ? "%s controller selected bulk mode is available" :
			"%s controller selected bulk mode is not available",
			selected[lpc3180_info->selected_controller]);

	return ERROR_OK;
}

static const struct command_registration lpc3180_exec_command_handlers[] = {
	{
		.name = "select",
		.handler = handle_lpc3180_select_command,
		.mode = COMMAND_EXEC,
		.help =
			"select MLC or SLC controller (default is MLC), SLC can be set to bulk mode",
		.usage = "bank_id ['mlc'|'slc' ['bulk'] ]",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration lpc3180_command_handler[] = {
	{
		.name = "lpc3180",
		.mode = COMMAND_ANY,
		.help = "LPC3180 NAND flash controller commands",
		.usage = "",
		.chain = lpc3180_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct nand_flash_controller lpc3180_nand_controller = {
	.name = "lpc3180",
	.commands = lpc3180_command_handler,
	.nand_device_command = lpc3180_nand_device_command,
	.init = lpc3180_init,
	.reset = lpc3180_reset,
	.command = lpc3180_command,
	.address = lpc3180_address,
	.write_data = lpc3180_write_data,
	.read_data = lpc3180_read_data,
	.write_page = lpc3180_write_page,
	.read_page = lpc3180_read_page,
	.nand_ready = lpc3180_nand_ready,
};
