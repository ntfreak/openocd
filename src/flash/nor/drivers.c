/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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

extern const struct flash_driver aduc702x_flash;
extern const struct flash_driver aducm360_flash;
extern const struct flash_driver ambiqmicro_flash;
extern const struct flash_driver at91sam3_flash;
extern const struct flash_driver at91sam4_flash;
extern const struct flash_driver at91sam4l_flash;
extern const struct flash_driver at91sam7_flash;
extern const struct flash_driver at91samd_flash;
extern const struct flash_driver ath79_flash;
extern const struct flash_driver atsame5_flash;
extern const struct flash_driver atsamv_flash;
extern const struct flash_driver avr_flash;
extern const struct flash_driver bluenrgx_flash;
extern const struct flash_driver cc3220sf_flash;
extern const struct flash_driver cc26xx_flash;
extern const struct flash_driver cfi_flash;
extern const struct flash_driver dsp5680xx_flash;
extern const struct flash_driver efm32_flash;
extern const struct flash_driver em357_flash;
extern const struct flash_driver esirisc_flash;
extern const struct flash_driver faux_flash;
extern const struct flash_driver fm3_flash;
extern const struct flash_driver fm4_flash;
extern const struct flash_driver fespi_flash;
extern const struct flash_driver jtagspi_flash;
extern const struct flash_driver kinetis_flash;
extern const struct flash_driver kinetis_ke_flash;
extern const struct flash_driver lpc2000_flash;
extern const struct flash_driver lpc288x_flash;
extern const struct flash_driver lpc2900_flash;
extern const struct flash_driver lpcspifi_flash;
extern const struct flash_driver max32xxx_flash;
extern const struct flash_driver mdr_flash;
extern const struct flash_driver mrvlqspi_flash;
extern const struct flash_driver msp432_flash;
extern const struct flash_driver niietcm4_flash;
extern const struct flash_driver nrf5_flash;
extern const struct flash_driver nrf51_flash;
extern const struct flash_driver numicro_flash;
extern const struct flash_driver ocl_flash;
extern const struct flash_driver pic32mx_flash;
extern const struct flash_driver psoc4_flash;
extern const struct flash_driver psoc5lp_flash;
extern const struct flash_driver psoc5lp_eeprom_flash;
extern const struct flash_driver psoc5lp_nvl_flash;
extern const struct flash_driver psoc6_flash;
extern const struct flash_driver sim3x_flash;
extern const struct flash_driver stellaris_flash;
extern const struct flash_driver stm32f1x_flash;
extern const struct flash_driver stm32f2x_flash;
extern const struct flash_driver stm32lx_flash;
extern const struct flash_driver stm32l4x_flash;
extern const struct flash_driver stm32h7x_flash;
extern const struct flash_driver stmsmi_flash;
extern const struct flash_driver str7x_flash;
extern const struct flash_driver str9x_flash;
extern const struct flash_driver str9xpec_flash;
extern const struct flash_driver swm050_flash;
extern const struct flash_driver tms470_flash;
extern const struct flash_driver virtual_flash;
extern const struct flash_driver w600_flash;
extern const struct flash_driver xcf_flash;
extern const struct flash_driver xmc1xxx_flash;
extern const struct flash_driver xmc4xxx_flash;

/**
 * The list of built-in flash drivers.
 * @todo Make this dynamically extendable with loadable modules.
 */
static const struct flash_driver * const flash_drivers[] = {
	&aduc702x_flash,
	&aducm360_flash,
	&ambiqmicro_flash,
	&at91sam3_flash,
	&at91sam4_flash,
	&at91sam4l_flash,
	&at91sam7_flash,
	&at91samd_flash,
	&ath79_flash,
	&atsame5_flash,
	&atsamv_flash,
	&avr_flash,
	&bluenrgx_flash,
	&cc3220sf_flash,
	&cc26xx_flash,
	&cfi_flash,
	&dsp5680xx_flash,
	&efm32_flash,
	&em357_flash,
	&esirisc_flash,
	&faux_flash,
	&fm3_flash,
	&fm4_flash,
	&fespi_flash,
	&jtagspi_flash,
	&kinetis_flash,
	&kinetis_ke_flash,
	&lpc2000_flash,
	&lpc288x_flash,
	&lpc2900_flash,
	&lpcspifi_flash,
	&max32xxx_flash,
	&mdr_flash,
	&mrvlqspi_flash,
	&msp432_flash,
	&niietcm4_flash,
	&nrf5_flash,
	&nrf51_flash,
	&numicro_flash,
	&ocl_flash,
	&pic32mx_flash,
	&psoc4_flash,
	&psoc5lp_flash,
	&psoc5lp_eeprom_flash,
	&psoc5lp_nvl_flash,
	&psoc6_flash,
	&sim3x_flash,
	&stellaris_flash,
	&stm32f1x_flash,
	&stm32f2x_flash,
	&stm32lx_flash,
	&stm32l4x_flash,
	&stm32h7x_flash,
	&stmsmi_flash,
	&str7x_flash,
	&str9x_flash,
	&str9xpec_flash,
	&swm050_flash,
	&tms470_flash,
	&virtual_flash,
	&xcf_flash,
	&xmc1xxx_flash,
	&xmc4xxx_flash,
	&w600_flash,
	NULL,
};

const struct flash_driver *flash_driver_find_by_name(const char *name)
{
	for (unsigned i = 0; flash_drivers[i]; i++) {
		if (strcmp(name, flash_drivers[i]->name) == 0)
			return flash_drivers[i];
	}
	return NULL;
}
