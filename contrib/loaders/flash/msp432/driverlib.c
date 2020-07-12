/******************************************************************************
*
* Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "driverlib.h"

/*
 * Wrapper function for the CPSID instruction.
 * Returns the state of PRIMASK on entry.
 */
uint32_t __attribute__((naked)) cpu_cpsid(void)
{
	uint32_t ret;

	/* Read PRIMASK and disable interrupts. */
	__asm("    mrs     r0, PRIMASK\n"
		  "    cpsid   i\n"
		  "    bx      lr\n"
			: "=r" (ret));

	/*
	 * The return is handled in the inline assembly, but the compiler will
	 * still complain if there is not an explicit return here (despite the fact
	 * that this does not result in any code being produced because of the
	 * naked attribute).
	 */
	return ret;
}

/* Wrapper function for the CPUWFI instruction. */
void __attribute__((naked)) cpu_wfi(void)
{
	/* Wait for the next interrupt. */
	__asm("    wfi\n"
		  "    bx      lr\n");
}

/* Power Control Module APIs */
#if defined(PCM)

static bool __pcm_set_core_voltage_level_advanced(uint_fast8_t voltage_level,
	uint32_t time_out, bool blocking)
{
	uint8_t power_mode;
	uint8_t current_voltage_level;
	uint32_t reg_value;
	bool bool_timeout;

	/* Getting current power mode and level */
	power_mode = pcm_get_power_mode();
	current_voltage_level = pcm_get_core_voltage_level();

	bool_timeout = time_out > 0 ? true : false;

	/* If we are already at the power mode they requested, return */
	if (current_voltage_level == voltage_level)
		return true;

	while (current_voltage_level != voltage_level) {

		reg_value = PCM->CTL0;

		switch (pcm_get_power_state()) {
			case PCM_AM_LF_VCORE1:
			case PCM_AM_DCDC_VCORE1:
			case PCM_AM_LDO_VCORE0:
				PCM->CTL0 = (PCM_KEY | (PCM_AM_LDO_VCORE1)
					| (reg_value & ~(PCM_CTL0_KEY_MASK | PCM_CTL0_AMR_MASK)));
				break;
			case PCM_AM_LF_VCORE0:
			case PCM_AM_DCDC_VCORE0:
			case PCM_AM_LDO_VCORE1:
				PCM->CTL0 = (PCM_KEY | (PCM_AM_LDO_VCORE0)
					| (reg_value & ~(PCM_CTL0_KEY_MASK | PCM_CTL0_AMR_MASK)));
				break;
			default:
				break;
		}

		if (blocking) {
			while (BITBAND_PERI(PCM->CTL1, PCM_CTL1_PMR_BUSY_OFS)) {
				if (bool_timeout && !(--time_out))
					return false;
			}
		} else
			return true;

		current_voltage_level = pcm_get_core_voltage_level();
	}

	/* Changing the power mode if we are stuck in LDO mode */
	if (power_mode != pcm_get_power_mode()) {
		if (power_mode == PCM_DCDC_MODE)
			return pcm_set_power_mode(PCM_DCDC_MODE);
		else
			return pcm_set_power_mode(PCM_LF_MODE);
	}

	return true;
}

bool pcm_set_core_voltage_level(uint_fast8_t voltage_level)
{
	return __pcm_set_core_voltage_level_advanced(voltage_level, 0, true);
}

uint8_t pcm_get_power_mode(void)
{
	uint8_t current_power_state;

	current_power_state = pcm_get_power_state();

	switch (current_power_state) {
		case PCM_AM_LDO_VCORE0:
		case PCM_AM_LDO_VCORE1:
		case PCM_LPM0_LDO_VCORE0:
		case PCM_LPM0_LDO_VCORE1:
		default:
			return PCM_LDO_MODE;
		case PCM_AM_DCDC_VCORE0:
		case PCM_AM_DCDC_VCORE1:
		case PCM_LPM0_DCDC_VCORE0:
		case PCM_LPM0_DCDC_VCORE1:
			return PCM_DCDC_MODE;
		case PCM_LPM0_LF_VCORE0:
		case PCM_LPM0_LF_VCORE1:
		case PCM_AM_LF_VCORE1:
		case PCM_AM_LF_VCORE0:
			return PCM_LF_MODE;
	}
}

uint8_t pcm_get_core_voltage_level(void)
{
	uint8_t current_power_state = pcm_get_power_state();

	switch (current_power_state) {
		case PCM_AM_LDO_VCORE0:
		case PCM_AM_DCDC_VCORE0:
		case PCM_AM_LF_VCORE0:
		case PCM_LPM0_LDO_VCORE0:
		case PCM_LPM0_DCDC_VCORE0:
		case PCM_LPM0_LF_VCORE0:
		default:
			return PCM_VCORE0;
		case PCM_AM_LDO_VCORE1:
		case PCM_AM_DCDC_VCORE1:
		case PCM_AM_LF_VCORE1:
		case PCM_LPM0_LDO_VCORE1:
		case PCM_LPM0_DCDC_VCORE1:
		case PCM_LPM0_LF_VCORE1:
			return PCM_VCORE1;
		case PCM_LPM3:
			return PCM_VCORELPM3;
	}
}

static bool __pcm_set_power_mode_advanced(uint_fast8_t power_mode,
	uint32_t time_out, bool blocking)
{
	uint8_t current_power_mode;
	uint8_t current_power_state;
	uint32_t reg_value;
	bool bool_timeout;

	/* Getting Current Power Mode */
	current_power_mode = pcm_get_power_mode();

	/* If the power mode being set it the same as the current mode, return */
	if (power_mode == current_power_mode)
		return true;

	current_power_state = pcm_get_power_state();

	bool_timeout = time_out > 0 ? true : false;

	/* Go through the while loop while we haven't achieved the power mode */
	while (current_power_mode != power_mode) {

		reg_value = PCM->CTL0;

		switch (current_power_state) {
			case PCM_AM_DCDC_VCORE0:
			case PCM_AM_LF_VCORE0:
				PCM->CTL0 = (PCM_KEY | PCM_AM_LDO_VCORE0
					| (reg_value & ~(PCM_CTL0_KEY_MASK | PCM_CTL0_AMR_MASK)));
				break;
			case PCM_AM_LF_VCORE1:
			case PCM_AM_DCDC_VCORE1:
				PCM->CTL0 = (PCM_KEY | PCM_AM_LDO_VCORE1
					| (reg_value & ~(PCM_CTL0_KEY_MASK | PCM_CTL0_AMR_MASK)));
				break;
			case PCM_AM_LDO_VCORE1: {
				if (power_mode == PCM_DCDC_MODE) {
					PCM->CTL0 = (PCM_KEY | PCM_AM_DCDC_VCORE1
						| (reg_value & ~(PCM_CTL0_KEY_MASK
						| PCM_CTL0_AMR_MASK)));
				} else if (power_mode == PCM_LF_MODE) {
					PCM->CTL0 = (PCM_KEY | PCM_AM_LF_VCORE1
						| (reg_value & ~(PCM_CTL0_KEY_MASK
						| PCM_CTL0_AMR_MASK)));
				} else
					return false;
				break;
			}
			case PCM_AM_LDO_VCORE0: {
				if (power_mode == PCM_DCDC_MODE) {
					PCM->CTL0 = (PCM_KEY | PCM_AM_DCDC_VCORE0
						| (reg_value & ~(PCM_CTL0_KEY_MASK
						| PCM_CTL0_AMR_MASK)));
				} else if (power_mode == PCM_LF_MODE) {
					PCM->CTL0 = (PCM_KEY | PCM_AM_LF_VCORE0
						| (reg_value & ~(PCM_CTL0_KEY_MASK
						| PCM_CTL0_AMR_MASK)));
				} else
					return false;
				break;
			}
			default:
				break;
		}

		if (blocking) {
			while (BITBAND_PERI(PCM->CTL1, PCM_CTL1_PMR_BUSY_OFS)) {
				if (bool_timeout && !(--time_out))
					return false;
			}
		} else
			return true;

		current_power_mode = pcm_get_power_mode();
		current_power_state = pcm_get_power_state();
	}

	return true;
}

bool pcm_set_power_mode(uint_fast8_t power_mode)
{
	return __pcm_set_power_mode_advanced(power_mode, 0, true);
}

static bool __pcm_set_power_state_advanced(uint_fast8_t power_state,
	uint32_t timeout, bool blocking)
{
	uint8_t current_power_state;
	current_power_state = pcm_get_power_state();

	if (current_power_state == power_state)
		return true;

	switch (power_state) {
		case PCM_AM_LDO_VCORE0:
			return __pcm_set_core_voltage_level_advanced(PCM_VCORE0, timeout,
					blocking) && __pcm_set_power_mode_advanced(PCM_LDO_MODE,
					timeout, blocking);
		case PCM_AM_LDO_VCORE1:
			return __pcm_set_core_voltage_level_advanced(PCM_VCORE1, timeout,
					blocking) && __pcm_set_power_mode_advanced(PCM_LDO_MODE,
					timeout, blocking);
		case PCM_AM_DCDC_VCORE0:
			return __pcm_set_core_voltage_level_advanced(PCM_VCORE0, timeout,
					blocking) && __pcm_set_power_mode_advanced(PCM_DCDC_MODE,
					timeout, blocking);
		case PCM_AM_DCDC_VCORE1:
			return __pcm_set_core_voltage_level_advanced(PCM_VCORE1, timeout,
					blocking) && __pcm_set_power_mode_advanced(PCM_DCDC_MODE,
					timeout, blocking);
		case PCM_AM_LF_VCORE0:
			return __pcm_set_core_voltage_level_advanced(PCM_VCORE0, timeout,
					blocking) && __pcm_set_power_mode_advanced(PCM_LF_MODE,
					timeout, blocking);
		case PCM_AM_LF_VCORE1:
			return __pcm_set_core_voltage_level_advanced(PCM_VCORE1, timeout,
					blocking) && __pcm_set_power_mode_advanced(PCM_LF_MODE,
					timeout, blocking);
		case PCM_LPM0_LDO_VCORE0:
			if (!__pcm_set_core_voltage_level_advanced(PCM_VCORE0, timeout,
				blocking) || !__pcm_set_power_mode_advanced(PCM_LDO_MODE,
				timeout, blocking))
				break;
			return pcm_goto_lpm0();
		case PCM_LPM0_LDO_VCORE1:
			if (!__pcm_set_core_voltage_level_advanced(PCM_VCORE1, timeout,
				blocking) || !__pcm_set_power_mode_advanced(PCM_LDO_MODE,
				timeout, blocking))
				break;
			return pcm_goto_lpm0();
		case PCM_LPM0_DCDC_VCORE0:
			if (!__pcm_set_core_voltage_level_advanced(PCM_VCORE0, timeout,
				blocking) || !__pcm_set_power_mode_advanced(PCM_DCDC_MODE,
				timeout, blocking))
				break;
			return pcm_goto_lpm0();
		case PCM_LPM0_DCDC_VCORE1:
			if (!__pcm_set_core_voltage_level_advanced(PCM_VCORE1, timeout,
				blocking) || !__pcm_set_power_mode_advanced(PCM_DCDC_MODE,
				timeout, blocking))
				break;
			return pcm_goto_lpm0();
		case PCM_LPM0_LF_VCORE0:
			if (!__pcm_set_core_voltage_level_advanced(PCM_VCORE0, timeout,
				blocking) || !__pcm_set_power_mode_advanced(PCM_LF_MODE,
				timeout, blocking))
				break;
			return pcm_goto_lpm0();
		case PCM_LPM0_LF_VCORE1:
			if (!__pcm_set_core_voltage_level_advanced(PCM_VCORE1, timeout,
				blocking) || !__pcm_set_power_mode_advanced(PCM_LF_MODE,
				timeout, blocking))
				break;
			return pcm_goto_lpm0();
		case PCM_LPM3:
			return pcm_goto_lpm3();
		case PCM_LPM4:
			return pcm_goto_lpm4();
		case PCM_LPM45:
			return pcm_shutdown_device(PCM_LPM45);
		case PCM_LPM35_VCORE0:
			return pcm_shutdown_device(PCM_LPM35_VCORE0);
		default:
			return false;
	}

	return false;
}

bool pcm_set_power_state(uint_fast8_t power_state)
{
	return __pcm_set_power_state_advanced(power_state, 0, true);
}

bool pcm_shutdown_device(uint32_t shutdown_mode)
{
	uint32_t shutdown_mode_bits = (shutdown_mode == PCM_LPM45) ?
		PCM_CTL0_LPMR_12 : PCM_CTL0_LPMR_10;

	/* If a power transition is occurring, return false */
	if (BITBAND_PERI(PCM->CTL1, PCM_CTL1_PMR_BUSY_OFS))
		return false;

	/* Initiating the shutdown */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_MSK;

	PCM->CTL0 = (PCM_KEY | shutdown_mode_bits
		| (PCM->CTL0 & ~(PCM_CTL0_KEY_MASK | PCM_CTL0_LPMR_MASK)));

	cpu_wfi();

	return true;
}

bool pcm_goto_lpm4(void)
{
	/* Disabling RTC_C and WDT_A */
	wdt_a_hold_timer();
	rtc_c_hold_clock();

	/* LPM4 is just LPM3 with WDT_A/RTC_C disabled... */
	return pcm_goto_lpm3();
}

bool pcm_goto_lpm0(void)
{
	/* If we are in the middle of a state transition, return false */
	if (BITBAND_PERI(PCM->CTL1, PCM_CTL1_PMR_BUSY_OFS))
		return false;

	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_MSK;

	cpu_wfi();

	return true;
}

bool pcm_goto_lpm3(void)
{
	uint_fast8_t current_power_state;
	uint_fast8_t current_power_mode;

	/* If we are in the middle of a state transition, return false */
	if (BITBAND_PERI(PCM->CTL1, PCM_CTL1_PMR_BUSY_OFS))
		return false;

	/* If we are in the middle of a shutdown, return false */
	if ((PCM->CTL0 & PCM_CTL0_LPMR_MASK) == PCM_CTL0_LPMR_10
		|| (PCM->CTL0 & PCM_CTL0_LPMR_MASK) == PCM_CTL0_LPMR_12)
		return false;

	current_power_mode = pcm_get_power_mode();
	current_power_state = pcm_get_power_state();

	if (current_power_mode == PCM_DCDC_MODE)
		pcm_set_power_mode(PCM_LDO_MODE);

	/* Clearing the SDR */
	PCM->CTL0 =
		(PCM->CTL0 & ~(PCM_CTL0_KEY_MASK | PCM_CTL0_LPMR_MASK)) | PCM_KEY;

	/* Setting the sleep deep bit */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_MSK;

	cpu_wfi();

	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_MSK;

	return pcm_set_power_state(current_power_state);
}

uint8_t pcm_get_power_state(void)
{
	return (PCM->CTL0 & PCM_CTL0_CPM_MASK) >> PCM_CTL0_CPM_OFS;
}

#endif

/* Real Time Clock APIs */
#if defined(RTC_C)

void rtc_c_hold_clock(void)
{
	RTC_C->CTL0 = (RTC_C->CTL0 & ~RTC_C_CTL0_KEY_MASK) | RTC_C_KEY;
	BITBAND_PERI(RTC_C->CTL13, RTC_C_CTL13_HOLD_OFS) = 1;
	BITBAND_PERI(RTC_C->CTL0, RTC_C_CTL0_KEY_OFS) = 0;
}

#endif

/* Watch Dog Timer APIs */
#if defined(WDT_A)

void wdt_a_hold_timer(void)
{
	/* Set Hold bit */
	uint8_t new_wdt_status = (WDT_A->CTL | WDT_A_CTL_HOLD);

	WDT_A->CTL = WDT_A_CTL_PW + new_wdt_status;
}

#endif
