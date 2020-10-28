/***************************************************************************
 *   Copyright (C) 2015 by Ivan Meleca                                     *
 *   ivan@artekit.eu                                                       *
 *                                                                         *
 *   Modified from kinetis.c                                               *
 *                                                                         *
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   kesmtp@freenet.de                                                     *
 *                                                                         *
 *   Copyright (C) 2011 sleep(5) ltd                                       *
 *   tomas@sleepfive.com                                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Christopher D. Kilgour                          *
 *   techie at whiterocker.com                                             *
 *                                                                         *
 *   Copyright (C) 2013 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   Copyright (C) 2015 Tomas Vanek                                        *
 *   vanekt@fbl.cz                                                         *
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

#include "jtag/interface.h"
#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>

/* Addresses */
#define SIM_SRSID					0x40048000
#define ICS_C1						0x40064000
#define ICS_C2						0x40064001
#define ICS_C3						0x40064002
#define ICS_C4						0x40064003
#define ICS_S						0x40064004
#define SIM_BUSDIV					0x40048018
#define SIM_CLKDIV_KE06				0x40048024
#define SIM_CLKDIV_KE04_44_64_80	0x40048024
#define SIM_CLKDIV_KE04_16_20_24	0x4004801C
#define WDOG_CS1					0x40052000

#define ICS_C2_BDIV_MASK			0xE0
#define ICS_C2_BDIV_SHIFT			5
#define ICS_C2_BDIV(x)				(((uint8_t)(((uint8_t)(x))<<ICS_C2_BDIV_SHIFT))&ICS_C2_BDIV_MASK)
#define ICS_S_LOCK_MASK				0x40
#define ICS_C4_SCFTRIM_MASK			0x1
#define SIM_CLKDIV_OUTDIV2_MASK		0x1000000
#define FTMRX_FCLKDIV_FDIV_MASK		0x3F
#define FTMRX_FCLKDIV_FDIV_SHIFT	0
#define FTMRX_FCLKDIV_FDIV(x)		(((uint8_t)(((uint8_t)(x))<<FTMRX_FCLKDIV_FDIV_SHIFT))&FTMRX_FCLKDIV_FDIV_MASK)
#define FTMRX_FCLKDIV_FDIVLCK_MASK	0x40
#define FTMRX_FCLKDIV_FDIVLCK_SHIFT	6
#define FTMRX_FCLKDIV_FDIVLD_MASK	0x80
#define FTMRX_FCLKDIV_FDIVLD_SHIFT	7
#define FTMRX_FSTAT_CCIF_MASK		0x80
#define FTMRX_FSTAT_MGSTAT0_MASK	0x01
#define FTMRX_FSTAT_MGSTAT1_MASK	0x02

/* Commands */
#define FTMRX_CMD_ALLERASED			0x01
#define FTMRX_CMD_BLOCKERASED		0x02
#define FTMRX_CMD_SECTIONERASED		0x03
#define FTMRX_CMD_READONCE			0x04
#define FTMRX_CMD_PROGFLASH			0x06
#define FTMRX_CMD_PROGONCE			0x07
#define FTMRX_CMD_ERASEALL			0x08
#define FTMRX_CMD_ERASEBLOCK		0x09
#define FTMRX_CMD_ERASESECTOR		0x0A
#define FTMRX_CMD_UNSECURE			0x0B
#define FTMRX_CMD_VERIFYACCESS		0x0C
#define FTMRX_CMD_SETMARGINLVL		0x0D
#define FTMRX_CMD_SETFACTORYLVL		0x0E
#define FTMRX_CMD_CONFIGNVM			0x0F

/* Error codes */
#define FTMRX_ERROR_ACCERR			0x20
#define FTMRX_ERROR_FPVIOL			0x10

#define KINETIS_KE_SRSID_FAMID(x)		((x >> 28) & 0x0F)
#define KINETIS_KE_SRSID_SUBFAMID(x)	((x >> 24) & 0x0F)
#define KINETIS_KE_SRSID_PINCOUNT(x)	((x >> 16) & 0x0F)

#define KINETIS_KE_SRSID_KEX2	0x02
#define KINETIS_KE_SRSID_KEX4	0x04
#define KINETIS_KE_SRSID_KEX6	0x06

struct kinetis_ke_flash_bank {
	uint32_t sector_size;
	uint32_t protection_size;

	uint32_t sim_srsid;
	uint32_t ftmrx_fclkdiv_addr;
	uint32_t ftmrx_fccobix_addr;
	uint32_t ftmrx_fstat_addr;
	uint32_t ftmrx_fprot_addr;
	uint32_t ftmrx_fccobhi_addr;
	uint32_t ftmrx_fccoblo_addr;
};

#define MDM_REG_STAT		0x00
#define MDM_REG_CTRL		0x04
#define MDM_REG_ID			0xfc

#define MDM_STAT_FMEACK		(1<<0)
#define MDM_STAT_FREADY		(1<<1)
#define MDM_STAT_SYSSEC		(1<<2)
#define MDM_STAT_SYSRES		(1<<3)
#define MDM_STAT_FMEEN		(1<<5)
#define MDM_STAT_BACKDOOREN	(1<<6)
#define MDM_STAT_LPEN		(1<<7)
#define MDM_STAT_VLPEN		(1<<8)
#define MDM_STAT_LLSMODEXIT	(1<<9)
#define MDM_STAT_VLLSXMODEXIT	(1<<10)
#define MDM_STAT_CORE_HALTED	(1<<16)
#define MDM_STAT_CORE_SLEEPDEEP	(1<<17)
#define MDM_STAT_CORESLEEPING	(1<<18)

#define MEM_CTRL_FMEIP		(1<<0)
#define MEM_CTRL_DBG_DIS	(1<<1)
#define MEM_CTRL_DBG_REQ	(1<<2)
#define MEM_CTRL_SYS_RES_REQ	(1<<3)
#define MEM_CTRL_CORE_HOLD_RES	(1<<4)
#define MEM_CTRL_VLLSX_DBG_REQ	(1<<5)
#define MEM_CTRL_VLLSX_DBG_ACK	(1<<6)
#define MEM_CTRL_VLLSX_STAT_ACK	(1<<7)

#define MDM_ACCESS_TIMEOUT	3000 /* iterations */

static int kinetis_ke_mdm_write_register(struct adiv5_dap *dap, unsigned reg, uint32_t value)
{
	int retval;
	LOG_DEBUG("MDM_REG[0x%02x] <- %08" PRIX32, reg, value);

	retval = dap_queue_ap_write(dap_ap(dap, 1), reg, value);
	if (retval != ERROR_OK) {
		LOG_DEBUG("MDM: failed to queue a write request");
		return retval;
	}

	retval = dap_run(dap);
	if (retval != ERROR_OK) {
		LOG_DEBUG("MDM: dap_run failed");
		return retval;
	}

	return ERROR_OK;
}

static int kinetis_ke_mdm_read_register(struct adiv5_dap *dap, unsigned reg, uint32_t *result)
{
	int retval;
	retval = dap_queue_ap_read(dap_ap(dap, 1), reg, result);
	if (retval != ERROR_OK) {
		LOG_DEBUG("MDM: failed to queue a read request");
		return retval;
	}

	retval = dap_run(dap);
	if (retval != ERROR_OK) {
		LOG_DEBUG("MDM: dap_run failed");
		return retval;
	}

	LOG_DEBUG("MDM_REG[0x%02x]: %08" PRIX32, reg, *result);
	return ERROR_OK;
}

static int kinetis_ke_mdm_poll_register(struct adiv5_dap *dap, unsigned reg, uint32_t mask, uint32_t value)
{
	uint32_t val;
	int retval;
	int timeout = MDM_ACCESS_TIMEOUT;

	do {
		retval = kinetis_ke_mdm_read_register(dap, reg, &val);
		if (retval != ERROR_OK || (val & mask) == value)
			return retval;

		alive_sleep(1);
	} while (timeout--);

	LOG_DEBUG("MDM: polling timed out");
	return ERROR_FAIL;
}

static int kinetis_ke_prepare_flash(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct kinetis_ke_flash_bank *kinfo = bank->driver_priv;
	uint8_t c2, c3, c4, s = 0;
	uint16_t trim_value = 0;
	uint16_t timeout = 0;
	uint32_t bus_clock = 0;
	uint32_t bus_reg_val = 0;
	uint32_t bus_reg_addr = 0;
	uint32_t flash_clk_div;
	uint8_t fclkdiv;
	int result;

	/*
	 * The RM states that the flash clock has to be set to 1MHz for writing and
	 * erasing operations (otherwise it can damage the flash).
	 * This function configures the entire clock tree to make sure we
	 * run at the specified clock. We'll set FEI mode running from the ~32KHz
	 * internal clock. So we need to:
	 * - Trim internal clock.
	 * - Configure the divider for ICSOUTCLK (ICS module).
	 * - Configure the divider to get a bus clock (SIM module).
	 * - Configure the flash clock that depends on the bus clock.
	 *
	 * For MKE02_40 and MKE02_20 we set ICSOUTCLK = 20MHz and bus clock = 20MHz.
	 * For MKE04 and MKE06 we run at ICSOUTCLK = 48MHz and bus clock = 24MHz.
	 */

	/*
	 * Trim internal clock
	 */
	switch (KINETIS_KE_SRSID_SUBFAMID(kinfo->sim_srsid)) {

		case KINETIS_KE_SRSID_KEX2:
			/* Both KE02_20 and KE02_40 should get the same trim value */
			trim_value = 0x4C;
			break;

		case KINETIS_KE_SRSID_KEX4:
			trim_value = 0x54;
			break;

		case KINETIS_KE_SRSID_KEX6:
			trim_value = 0x58;
			break;
	}

	result = target_read_u8(target, ICS_C4, &c4);
	if (result != ERROR_OK)
		return result;

	c3 = trim_value;
	c4 = (c4 & ~(ICS_C4_SCFTRIM_MASK)) | ((trim_value >> 8) & 0x01);

	result = target_write_u8(target, ICS_C3, c3);
	if (result != ERROR_OK)
		return result;

	result = target_write_u8(target, ICS_C4, c4);
	if (result != ERROR_OK)
		return result;

	result = target_read_u8(target, ICS_S, &s);
	if (result != ERROR_OK)
		return result;

	/* Wait */
	while (!(s & ICS_S_LOCK_MASK)) {

		if (timeout <= 1000) {
			timeout++;
			alive_sleep(1);
		} else {
			return ERROR_FAIL;
		}

		result = target_read_u8(target, ICS_S, &s);
		if (result != ERROR_OK)
			return result;
	}

	/* ... trim done ... */

	/*
	 * Configure SIM (bus clock)
	 */
	switch (KINETIS_KE_SRSID_SUBFAMID(kinfo->sim_srsid)) {

		/* KE02 sub-family operates on SIM_BUSDIV */
		case KINETIS_KE_SRSID_KEX2:
			bus_reg_val = 0;
			bus_reg_addr = SIM_BUSDIV;
			bus_clock = 20000000;
			break;

		/* KE04 and KE06 sub-family operates on SIM_CLKDIV
		 * Clocks are divided by:
		 * DIV1 = core clock = 48MHz
		 * DIV2 = bus clock = 24Mhz
		 * DIV3 = timer clocks
		 * So we need to configure SIM_CLKDIV, DIV1 and DIV2 value
		 */
		case KINETIS_KE_SRSID_KEX4:
			/* KE04 devices have the SIM_CLKDIV register at a different offset
			 * depending on the pin count. */
			switch (KINETIS_KE_SRSID_PINCOUNT(kinfo->sim_srsid)) {

				/* 16, 20 and 24 pins */
				case 1:
				case 2:
				case 3:
					bus_reg_addr = SIM_CLKDIV_KE04_16_20_24;
					break;

				/* 44, 64 and 80 pins */
				case 5:
				case 7:
				case 8:
					bus_reg_addr = SIM_CLKDIV_KE04_44_64_80;
					break;

				default:
					LOG_ERROR("KE04 - Unknown pin count");
					return ERROR_FAIL;
			}

			bus_reg_val = SIM_CLKDIV_OUTDIV2_MASK;
			bus_clock = 24000000;
			break;

		case KINETIS_KE_SRSID_KEX6:
			bus_reg_val = SIM_CLKDIV_OUTDIV2_MASK;
			bus_reg_addr = SIM_CLKDIV_KE06;
			bus_clock = 24000000;
			break;
	}

	result = target_write_u32(target, bus_reg_addr, bus_reg_val);
	if (result != ERROR_OK)
		return result;

	/*
	 * Configure ICS to FEI (internal source)
	 */
	result = target_read_u8(target, ICS_C2, &c2);
	if (result != ERROR_OK)
		return result;

	c2 &= ~ICS_C2_BDIV_MASK;

	switch (KINETIS_KE_SRSID_SUBFAMID(kinfo->sim_srsid)) {

		case KINETIS_KE_SRSID_KEX2:
			/* Note: since there are two KE02 types, the KE02_40 @ 40MHz and the
			 * KE02_20 @ 20MHz, we divide here the ~40MHz ICSFLLCLK down to 20MHz,
			 * for compatibility.
			 */
			c2 |= ICS_C2_BDIV(1);
			break;

		case KINETIS_KE_SRSID_KEX4:
		case KINETIS_KE_SRSID_KEX6:
			/* For KE04 and KE06, the ICSFLLCLK can be 48MHz. */
			c2 |= ICS_C2_BDIV(0);
			break;
	}

	result = target_write_u8(target, ICS_C2, c2);
	if (result != ERROR_OK)
		return result;

	/* Internal clock as reference (IREFS = 1) */
	result = target_write_u8(target, ICS_C1, 4);
	if (result != ERROR_OK)
		return result;

	/* Wait for FLL to lock */
	result = target_read_u8(target, ICS_S, &s);
	if (result != ERROR_OK)
		return result;

	while (!(s & ICS_S_LOCK_MASK)) {

		if (timeout <= 1000) {
			timeout++;
			alive_sleep(1);
		} else {
			return ERROR_FLASH_OPERATION_FAILED;
		}

		result = target_read_u8(target, ICS_S, &s);
		if (result != ERROR_OK)
			return result;
	}

	/*
	 * Configure flash clock to 1MHz.
	 */
	flash_clk_div = bus_clock / 1000000L - 1;

	/* Check if the FCLKDIV register is locked */
	result = target_read_u8(target, kinfo->ftmrx_fclkdiv_addr, &fclkdiv);
	if (result != ERROR_OK)
		return result;

	if (!(fclkdiv & FTMRX_FCLKDIV_FDIVLCK_MASK)) {
		/* Unlocked. Check if the register was configured, and if so, if it has the right value */
		if ((fclkdiv & FTMRX_FCLKDIV_FDIVLD_MASK) &&
			((fclkdiv & FTMRX_FCLKDIV_FDIV_MASK) != FTMRX_FCLKDIV_FDIV(flash_clk_div))) {
			LOG_WARNING("Flash clock was already set and contains an invalid value.");
			LOG_WARNING("Please reset the target.");
			return ERROR_FAIL;
		}

		/* Finally, configure the flash clock */
		fclkdiv = (fclkdiv & ~(FTMRX_FCLKDIV_FDIV_MASK)) | FTMRX_FCLKDIV_FDIV(flash_clk_div);
		result = target_write_u8(target, kinfo->ftmrx_fclkdiv_addr, fclkdiv);
		if (result != ERROR_OK)
			return result;
	} else {
		/* Locked. Check if the current value is correct. */
		if ((fclkdiv & FTMRX_FCLKDIV_FDIV_MASK) != FTMRX_FCLKDIV_FDIV(flash_clk_div)) {
			LOG_WARNING("Flash clock register is locked and contains an invalid value.");
			LOG_WARNING("Please reset the target.");
			return ERROR_FAIL;
		}
	}

	LOG_INFO("Flash clock ready");
	return ERROR_OK;
}

static int kinetis_ke_stop_watchdog(struct target *target)
{
	struct working_area *watchdog_algorithm;
	struct armv7m_algorithm armv7m_info;
	int retval;
	uint8_t cs1;

	static const uint8_t watchdog_code[] = {
#include "../../../contrib/loaders/flash/kinetis_ke/kinetis_ke_watchdog.inc"
	};

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Check if the watchdog is enabled */
	retval = target_read_u8(target, WDOG_CS1, &cs1);
	if (retval != ERROR_OK)
		return retval;

	if (!(cs1 & 0x80)) {
		/* Already stopped */
		return ERROR_OK;
	}

	/* allocate working area with watchdog code */
	if (target_alloc_working_area(target, sizeof(watchdog_code), &watchdog_algorithm) != ERROR_OK) {
		LOG_WARNING("No working area available for watchdog algorithm");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, watchdog_algorithm->address,
			sizeof(watchdog_code), watchdog_code);
	if (retval != ERROR_OK)
		return retval;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_algorithm(target, 0, NULL, 0, NULL,
			watchdog_algorithm->address, 0, 100000, &armv7m_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error executing Kinetis KE watchdog algorithm");
	} else {
		LOG_INFO("Watchdog stopped");
	}

	target_free_working_area(target, watchdog_algorithm);

	return retval;
}

COMMAND_HANDLER(kinetis_ke_disable_wdog_handler)
{
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return kinetis_ke_stop_watchdog(target);
}

COMMAND_HANDLER(kinetis_ke_mdm_mass_erase)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct adiv5_dap *dap = cortex_m->armv7m.arm.dap;

	if (!dap) {
		LOG_ERROR("Cannot perform mass erase with a high-level adapter");
		return ERROR_FAIL;
	}

	int retval;

	/* According to chapter 18.3.7.2 of the KE02 reference manual */

	/* assert SRST */
	if (jtag_get_reset_config() & RESET_HAS_SRST)
		adapter_assert_reset();

	/*
	 * 1. Reset the device by asserting RESET pin or DAP_CTRL[3]
	 */
	retval = kinetis_ke_mdm_write_register(dap, MDM_REG_CTRL, MEM_CTRL_SYS_RES_REQ);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * ... Read the MDM-AP status register until the Flash Ready bit sets...
	 */
	retval = kinetis_ke_mdm_poll_register(dap, MDM_REG_STAT,
					   MDM_STAT_FREADY | MDM_STAT_SYSRES,
					   MDM_STAT_FREADY);
	if (retval != ERROR_OK) {
		LOG_ERROR("MDM : flash ready timeout");
		return retval;
	}

	/*
	 * 2. Set DAP_CTRL[0] bit to invoke debug mass erase via SWD
	 * 3. Release reset by deasserting RESET pin or DAP_CTRL[3] bit via SWD.
	 */
	retval = kinetis_ke_mdm_write_register(dap, MDM_REG_CTRL, MEM_CTRL_FMEIP);
	if (retval != ERROR_OK)
		return retval;

	/* As a sanity check make sure that device started mass erase procedure */
	retval = kinetis_ke_mdm_poll_register(dap, MDM_REG_STAT,
					   MDM_STAT_FMEACK, MDM_STAT_FMEACK);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * 4. Wait till DAP_CTRL[0] bit is cleared (after mass erase completes,
	 * DAP_CTRL[0] bit is cleared automatically).
	 */
	retval = kinetis_ke_mdm_poll_register(dap, MDM_REG_CTRL,
					   MEM_CTRL_FMEIP,
					   0);
	if (retval != ERROR_OK)
		return retval;

	if (jtag_get_reset_config() & RESET_HAS_SRST)
		adapter_deassert_reset();

	return ERROR_OK;
}

static const uint32_t kinetis_ke_known_mdm_ids[] = {
	0x001C0020,	/* Kinetis-L/M/V/E/KE Series */
};

/*
 * This function implements the procedure to connect to
 * SWD/JTAG on Kinetis K and L series of devices as it is described in
 * AN4835 "Production Flash Programming Best Practices for Kinetis K-
 * and L-series MCUs" Section 4.1.1
 */
COMMAND_HANDLER(kinetis_ke_check_flash_security_status)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);
	struct adiv5_dap *dap = cortex_m->armv7m.arm.dap;

	if (!dap) {
		LOG_WARNING("Cannot check flash security status with a high-level adapter");
		return ERROR_OK;
	}

	uint32_t val;
	int retval;

	/*
	 * ... The MDM-AP ID register can be read to verify that the
	 * connection is working correctly...
	 */
	retval = kinetis_ke_mdm_read_register(dap, MDM_REG_ID, &val);
	if (retval != ERROR_OK) {
		LOG_ERROR("MDM: failed to read ID register");
		goto fail;
	}

	bool found = false;
	for (size_t i = 0; i < ARRAY_SIZE(kinetis_ke_known_mdm_ids); i++) {
		if (val == kinetis_ke_known_mdm_ids[i]) {
			found = true;
			break;
		}
	}

	if (!found)
		LOG_WARNING("MDM: unknown ID %08" PRIX32, val);

	/*
	 * ... Read the MDM-AP status register until the Flash Ready bit sets...
	 */
	retval = kinetis_ke_mdm_poll_register(dap, MDM_REG_STAT,
					   MDM_STAT_FREADY,
					   MDM_STAT_FREADY);
	if (retval != ERROR_OK) {
		LOG_ERROR("MDM: flash ready timeout");
		goto fail;
	}

	/*
	 * ... Read the System Security bit to determine if security is enabled.
	 * If System Security = 0, then proceed. If System Security = 1, then
	 * communication with the internals of the processor, including the
	 * flash, will not be possible without issuing a mass erase command or
	 * unsecuring the part through other means (backdoor key unlock)...
	 */
	retval = kinetis_ke_mdm_read_register(dap, MDM_REG_STAT, &val);
	if (retval != ERROR_OK) {
		LOG_ERROR("MDM: failed to read MDM_REG_STAT");
		goto fail;
	}

	if (val & MDM_STAT_SYSSEC) {
		jtag_poll_set_enabled(false);

		LOG_WARNING("*********** ATTENTION! ATTENTION! ATTENTION! ATTENTION! **********");
		LOG_WARNING("****                                                          ****");
		LOG_WARNING("**** Your Kinetis MCU is in secured state, which means that,  ****");
		LOG_WARNING("**** with exception for very basic communication, JTAG/SWD    ****");
		LOG_WARNING("**** interface will NOT work. In order to restore its         ****");
		LOG_WARNING("**** functionality please issue 'kinetis_ke mdm mass_erase'   ****");
		LOG_WARNING("**** command, power cycle the MCU and restart OpenOCD.        ****");
		LOG_WARNING("****                                                          ****");
		LOG_WARNING("*********** ATTENTION! ATTENTION! ATTENTION! ATTENTION! **********");
	} else {
		LOG_INFO("MDM: Chip is unsecured. Continuing.");
		jtag_poll_set_enabled(true);
	}

	return ERROR_OK;

fail:
	LOG_ERROR("MDM: Failed to check security status of the MCU. Cannot proceed further");
	jtag_poll_set_enabled(false);
	return retval;
}

FLASH_BANK_COMMAND_HANDLER(kinetis_ke_flash_bank_command)
{
	struct kinetis_ke_flash_bank *bank_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_INFO("add flash_bank kinetis_ke %s", bank->name);

	bank_info = malloc(sizeof(struct kinetis_ke_flash_bank));

	memset(bank_info, 0, sizeof(struct kinetis_ke_flash_bank));

	bank->driver_priv = bank_info;

	return ERROR_OK;
}

/* Kinetis Program-LongWord Microcodes */
static uint8_t kinetis_ke_flash_write_code[] = {
#include "../../../contrib/loaders/flash/kinetis_ke/kinetis_ke_flash.inc"
};

static int kinetis_ke_write_words(struct flash_bank *bank, const uint8_t *buffer,
								uint32_t offset, uint32_t words)
{
	struct kinetis_ke_flash_bank *kinfo = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t ram_buffer_size = 512 + 16;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;
	uint32_t flash_code_size;

	LOG_INFO("Kinetis KE: FLASH Write ...");

	/* allocate working area with flash programming code */
	if (target_alloc_working_area(target, sizeof(kinetis_ke_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Patch the FTMRx registers addresses */
	flash_code_size = sizeof(kinetis_ke_flash_write_code);
	buf_set_u32(&kinetis_ke_flash_write_code[flash_code_size-16], 0, 32, kinfo->ftmrx_fstat_addr);
	buf_set_u32(&kinetis_ke_flash_write_code[flash_code_size-12], 0, 32, kinfo->ftmrx_fccobix_addr);
	buf_set_u32(&kinetis_ke_flash_write_code[flash_code_size-8], 0, 32, kinfo->ftmrx_fccobhi_addr);
	buf_set_u32(&kinetis_ke_flash_write_code[flash_code_size-4], 0, 32, kinfo->ftmrx_fccoblo_addr);

	retval = target_write_buffer(target, write_algorithm->address,
		sizeof(kinetis_ke_flash_write_code), kinetis_ke_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	if (target_alloc_working_area(target, ram_buffer_size, &source) != ERROR_OK) {
		/* free working area, write algorithm already allocated */
		target_free_working_area(target, write_algorithm);

		LOG_WARNING("No large enough working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, address);
	buf_set_u32(reg_params[1].value, 0, 32, words);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);

	retval = target_run_flash_async_algorithm(target, buffer, words, 4,
			0, NULL,
			4, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		if (buf_get_u32(reg_params[0].value, 0, 32) & FTMRX_ERROR_ACCERR)
			LOG_ERROR("flash access error");

		if (buf_get_u32(reg_params[0].value, 0, 32) & FTMRX_ERROR_FPVIOL)
			LOG_ERROR("flash protection violation");
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	return retval;
}

static int kinetis_ke_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	LOG_WARNING("kinetis_ke_protect not supported yet");
	/* FIXME: TODO */

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_FLASH_BANK_INVALID;
}

static int kinetis_ke_protect_check(struct flash_bank *bank)
{
	struct kinetis_ke_flash_bank *kinfo = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int result;
	uint8_t fprot;
	uint8_t fpopen, fpldis, fphdis;
	uint8_t fphs, fpls;
	uint32_t lprot_size = 0, hprot_size = 0;
	uint32_t lprot_to = 0, hprot_from = 0;

	/* read protection register */
	result = target_read_u8(bank->target, kinfo->ftmrx_fprot_addr, &fprot);

	if (result != ERROR_OK)
		return result;

	fpopen = fprot & 0x80;
	fpldis = fprot & 0x04;
	fphdis = fprot & 0x20;
	fphs = (fprot >> 3) & 0x03;
	fpls = fprot & 0x03;

	/* Fully unprotected? */
	if (fpopen && fpldis && fphdis) {
		LOG_WARNING("No flash protection found.");

		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_protected = 0;

		kinfo->protection_size = 0;
	} else {
		LOG_WARNING("Flash protected. FPOPEN=%i FPLDIS=%i FPHDIS=%i FPLS=%i FPHS=%i",
					fpopen ? 1 : 0, fpldis ? 1 : 0, fphdis ? 1 : 0, fpls, fphs);

		/* Retrieve which region is protected and how much */
		if (fpopen) {
			if (fpldis == 0)
				lprot_size = (kinfo->sector_size * 4) << fpls;

			if (fphdis == 0)
				hprot_size = (kinfo->sector_size * 2) << fphs;
		} else {
			if (fpldis == 1)
				lprot_size = (kinfo->sector_size * 4) << fpls;

			if (fphdis == 1)
				hprot_size = (kinfo->sector_size * 2) << fphs;
		}

		kinfo->protection_size = lprot_size + hprot_size;

		/* lprot_to indicates up to where the lower region is protected */
		lprot_to = lprot_size / kinfo->sector_size;

		/* hprot_from indicates from where the upper region is protected */
		hprot_from = (0x8000 - hprot_size) / kinfo->sector_size;

		for (unsigned int i = 0; i < bank->num_sectors; i++) {

			/* Check if the sector is in the lower region */
			if (bank->sectors[i].offset < 0x4000) {
				/* Compare the sector start address against lprot_to */
				if (lprot_to && (i < lprot_to))
					bank->sectors[i].is_protected = 1;
				else
					bank->sectors[i].is_protected = 0;

			/* Check if the sector is between the lower and upper region
			 * OR after the upper region */
			} else if (bank->sectors[i].offset < 0x6000 || bank->sectors[i].offset >= 0x8000) {
				/* If fpopen is 1 then these regions are protected */
				if (fpopen)
					bank->sectors[i].is_protected = 0;
				else
					bank->sectors[i].is_protected = 1;

			/* Check if the sector is in the upper region */
			} else if (bank->sectors[i].offset < 0x8000) {
				if (hprot_from && (i > hprot_from))
					bank->sectors[i].is_protected = 1;
				else
					bank->sectors[i].is_protected = 0;
			}
		}
	}

	return ERROR_OK;
}

static int kinetis_ke_ftmrx_command(struct flash_bank *bank, uint8_t count,
									uint8_t *FCCOBIX, uint8_t *FCCOBHI, uint8_t *FCCOBLO, uint8_t *fstat)
{
	uint8_t i;
	int result;
	struct target *target = bank->target;
	struct kinetis_ke_flash_bank *kinfo = bank->driver_priv;
	uint32_t timeout = 0;

	/* Clear error flags */
	result = target_write_u8(target, kinfo->ftmrx_fstat_addr, 0x30);
	if (result != ERROR_OK)
		return result;

	for (i = 0; i < count; i++)	{
		/* Write index */
		result = target_write_u8(target, kinfo->ftmrx_fccobix_addr, FCCOBIX[i]);
		if (result != ERROR_OK)
			return result;

		/* Write high part */
		result = target_write_u8(target, kinfo->ftmrx_fccobhi_addr, FCCOBHI[i]);
		if (result != ERROR_OK)
			return result;

		/* Write low part (that is not always required) */
		if (FCCOBLO) {
			result = target_write_u8(target, kinfo->ftmrx_fccoblo_addr, FCCOBLO[i]);
			if (result != ERROR_OK)
				return result;
		}
	}

	/* Launch the command */
	result = target_write_u8(target, kinfo->ftmrx_fstat_addr, 0x80);
	if (result != ERROR_OK)
		return result;

	/* Wait for it to finish */
	result = target_read_u8(target, kinfo->ftmrx_fstat_addr, fstat);
	if (result != ERROR_OK)
		return result;

	while (!(*fstat & FTMRX_FSTAT_CCIF_MASK)) {
		if (timeout <= 1000) {
			timeout++;
			alive_sleep(1);
		} else {
			return ERROR_FLASH_OPERATION_FAILED;
		}

		result = target_read_u8(target, kinfo->ftmrx_fstat_addr, fstat);
		if (result != ERROR_OK)
			return result;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(kinetis_ke_securing_test)
{
	int result;
	struct target *target = get_current_target(CMD_CTX);
	struct flash_bank *bank = NULL;
	uint32_t address;

	uint8_t FCCOBIX[2], FCCOBHI[2], FCCOBLO[2], fstat;

	result = get_flash_bank_by_addr(target, 0x00000000, true, &bank);
	if (result != ERROR_OK)
		return result;

	assert(bank != NULL);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	address = bank->base + 0x00000400;

	FCCOBIX[0] = 0;
	FCCOBHI[0] = FTMRX_CMD_ERASESECTOR;
	FCCOBLO[0] = address >> 16;

	FCCOBIX[1] = 1;
	FCCOBHI[1] = address >> 8;
	FCCOBLO[1] = address;

	return kinetis_ke_ftmrx_command(bank, 2, FCCOBIX, FCCOBHI, FCCOBLO, &fstat);
}

static int kinetis_ke_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int result;
	uint8_t FCCOBIX[2], FCCOBHI[2], FCCOBLO[2], fstat;
	bool fcf_erased = false;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first > bank->num_sectors) || (last > bank->num_sectors))
		return ERROR_FLASH_OPERATION_FAILED;

	result = kinetis_ke_prepare_flash(bank);
	if (result != ERROR_OK)
		return result;

	for (unsigned int i = first; i <= last; i++) {
		FCCOBIX[0] = 0;
		FCCOBHI[0] = FTMRX_CMD_ERASESECTOR;
		FCCOBLO[0] = (bank->base + bank->sectors[i].offset) >> 16;

		FCCOBIX[1] = 1;
		FCCOBHI[1] = (bank->base + bank->sectors[i].offset) >> 8;
		FCCOBLO[1] = (bank->base + bank->sectors[i].offset);

		result = kinetis_ke_ftmrx_command(bank, 2, FCCOBIX, FCCOBHI, FCCOBLO, &fstat);

		if (result != ERROR_OK)	{
			LOG_WARNING("erase sector %u failed", i);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bank->sectors[i].is_erased = 1;

		if (i == 2)
			fcf_erased = true;
	}

	if (fcf_erased) {
		LOG_WARNING
			("flash configuration field erased, please reset the device");
	}

	return ERROR_OK;
}

static int kinetis_ke_write(struct flash_bank *bank, const uint8_t *buffer,
			 uint32_t offset, uint32_t count)
{
	int result;
	uint8_t *new_buffer = NULL;
	uint32_t words = count / 4;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset > bank->size)
		return ERROR_FLASH_BANK_INVALID;

	if (offset & 0x3) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks the required alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	result = kinetis_ke_stop_watchdog(bank->target);
	if (result != ERROR_OK)
			return result;

	result = kinetis_ke_prepare_flash(bank);
	if (result != ERROR_OK)
		return result;

	if (count & 0x3) {
		uint32_t old_count = count;
		count = (old_count | 3) + 1;
		new_buffer = malloc(count);
		if (new_buffer == NULL) {
			LOG_ERROR("odd number of bytes to write and no memory "
				"for padding buffer");
			return ERROR_FAIL;
		}

		LOG_INFO("odd number of bytes to write (%" PRIu32 "), extending to %" PRIu32 " "
			"and padding with 0xff", old_count, count);

		memset(new_buffer, 0xff, count);
		buffer = memcpy(new_buffer, buffer, old_count);
		words++;
	}

	result = kinetis_ke_write_words(bank, buffer, offset, words);
	free(new_buffer);

	return result;
}

static int kinetis_ke_probe(struct flash_bank *bank)
{
	int result;
	uint32_t offset = 0;
	struct target *target = bank->target;
	struct kinetis_ke_flash_bank *kinfo = bank->driver_priv;

	result = target_read_u32(target, SIM_SRSID, &kinfo->sim_srsid);
	if (result != ERROR_OK)
		return result;

	if (KINETIS_KE_SRSID_FAMID(kinfo->sim_srsid) != 0x00) {
		LOG_ERROR("Unsupported KE family");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	switch (KINETIS_KE_SRSID_SUBFAMID(kinfo->sim_srsid)) {
		case KINETIS_KE_SRSID_KEX2:
			LOG_INFO("KE02 sub-family");
			break;

		case KINETIS_KE_SRSID_KEX4:
			LOG_INFO("KE04 sub-family");
			break;

		case KINETIS_KE_SRSID_KEX6:
			LOG_INFO("KE06 sub-family");
			break;

		default:
			LOG_ERROR("Unsupported KE sub-family");
			return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	/* We can only retrieve the ke0x part, but there is no way to know
	 * the flash size, so assume the maximum flash size for the entire
	 * sub family.
	 */
	bank->base = 0x00000000;
	kinfo->sector_size = 512;

	switch (KINETIS_KE_SRSID_SUBFAMID(kinfo->sim_srsid)) {

		case KINETIS_KE_SRSID_KEX2:
			/* Max. 64KB */
			bank->size = 0x00010000;
			bank->num_sectors = 128;

			/* KE02 uses the FTMRH flash controller,
			 * and registers have a different offset from the
			 * FTMRE flash controller. Sort this out here.
			 */
			kinfo->ftmrx_fclkdiv_addr = 0x40020000;
			kinfo->ftmrx_fccobix_addr = 0x40020002;
			kinfo->ftmrx_fstat_addr = 0x40020006;
			kinfo->ftmrx_fprot_addr = 0x40020008;
			kinfo->ftmrx_fccobhi_addr = 0x4002000A;
			kinfo->ftmrx_fccoblo_addr = 0x4002000B;
			break;

		case KINETIS_KE_SRSID_KEX6:
		case KINETIS_KE_SRSID_KEX4:
			/* Max. 128KB */
			bank->size = 0x00020000;
			bank->num_sectors = 256;

			/* KE04 and KE06 use the FTMRE flash controller,
			 * and registers have a different offset from the
			 * FTMRH flash controller. Sort this out here.
			 */
			kinfo->ftmrx_fclkdiv_addr = 0x40020003;
			kinfo->ftmrx_fccobix_addr = 0x40020001;
			kinfo->ftmrx_fstat_addr = 0x40020005;
			kinfo->ftmrx_fprot_addr = 0x4002000B;
			kinfo->ftmrx_fccobhi_addr = 0x40020009;
			kinfo->ftmrx_fccoblo_addr = 0x40020008;
			break;
	}

	free(bank->sectors);

	assert(bank->num_sectors > 0);
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = kinfo->sector_size;
		offset += kinfo->sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	return ERROR_OK;
}

static int kinetis_ke_auto_probe(struct flash_bank *bank)
{
	struct kinetis_ke_flash_bank *kinfo = bank->driver_priv;

	if (kinfo->sim_srsid)
		return ERROR_OK;

	return kinetis_ke_probe(bank);
}

static int kinetis_ke_info(struct flash_bank *bank, char *buf, int buf_size)
{
	(void) snprintf(buf, buf_size,
			"%s driver for flash bank %s at " TARGET_ADDR_FMT,
			bank->driver->name,	bank->name, bank->base);

	return ERROR_OK;
}

static int kinetis_ke_blank_check(struct flash_bank *bank)
{
	uint8_t FCCOBIX[3], FCCOBHI[3], FCCOBLO[3], fstat;
	uint16_t longwords = 0;
	int result;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	result = kinetis_ke_prepare_flash(bank);
	if (result != ERROR_OK)
		return result;

	/* check if whole bank is blank */
	FCCOBIX[0] = 0;
	FCCOBHI[0] = FTMRX_CMD_ALLERASED;

	result = kinetis_ke_ftmrx_command(bank, 1, FCCOBIX, FCCOBHI, NULL, &fstat);

	if (result != ERROR_OK)
		return result;

	if (fstat & (FTMRX_FSTAT_MGSTAT0_MASK | FTMRX_FSTAT_MGSTAT1_MASK)) {
		/* the whole bank is not erased, check sector-by-sector */
		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			FCCOBIX[0] = 0;
			FCCOBHI[0] = FTMRX_CMD_SECTIONERASED;
			FCCOBLO[0] = (bank->base + bank->sectors[i].offset) >> 16;

			FCCOBIX[1] = 1;
			FCCOBHI[1] = (bank->base + bank->sectors[i].offset) >> 8;
			FCCOBLO[1] = (bank->base + bank->sectors[i].offset);

			longwords = 128;

			FCCOBIX[2] = 2;
			FCCOBHI[2] = longwords >> 8;
			FCCOBLO[2] = longwords;

			result = kinetis_ke_ftmrx_command(bank, 3, FCCOBIX, FCCOBHI, FCCOBLO, &fstat);

			if (result == ERROR_OK)	{
				bank->sectors[i].is_erased = !(fstat & (FTMRX_FSTAT_MGSTAT0_MASK | FTMRX_FSTAT_MGSTAT1_MASK));
			} else {
				LOG_DEBUG("Ignoring error on PFlash sector blank-check");
				bank->sectors[i].is_erased = -1;
			}
		}
	} else {
		/* the whole bank is erased, update all sectors */
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static const struct command_registration kinetis_ke_security_command_handlers[] = {
	{
		.name = "check_security",
		.mode = COMMAND_EXEC,
		.help = "",
		.usage = "",
		.handler = kinetis_ke_check_flash_security_status,
	},
	{
		.name = "mass_erase",
		.mode = COMMAND_EXEC,
		.help = "",
		.usage = "",
		.handler = kinetis_ke_mdm_mass_erase,
	},
	{
		.name = "test_securing",
		.mode = COMMAND_EXEC,
		.help = "",
		.usage = "",
		.handler = kinetis_ke_securing_test,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration kinetis_ke_exec_command_handlers[] = {
	{
		.name = "mdm",
		.mode = COMMAND_ANY,
		.help = "",
		.usage = "",
		.chain = kinetis_ke_security_command_handlers,
	},
	{
		.name = "disable_wdog",
		.mode = COMMAND_EXEC,
		.help = "Disable the watchdog timer",
		.usage = "",
		.handler = kinetis_ke_disable_wdog_handler,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration kinetis_ke_command_handler[] = {
	{
		.name = "kinetis_ke",
		.mode = COMMAND_ANY,
		.help = "Kinetis KE NAND flash controller commands",
		.usage = "",
		.chain = kinetis_ke_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver kinetis_ke_flash = {
	.name = "kinetis_ke",
	.commands = kinetis_ke_command_handler,
	.flash_bank_command = kinetis_ke_flash_bank_command,
	.erase = kinetis_ke_erase,
	.protect = kinetis_ke_protect,
	.write = kinetis_ke_write,
	.read = default_flash_read,
	.probe = kinetis_ke_probe,
	.auto_probe = kinetis_ke_auto_probe,
	.erase_check = kinetis_ke_blank_check,
	.protect_check = kinetis_ke_protect_check,
	.info = kinetis_ke_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
