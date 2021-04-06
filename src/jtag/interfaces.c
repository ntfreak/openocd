/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 *                                                                         *
 *   Copyright (C) 2020, Ampere Computing LLC                              *
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

#include "interfaces.h"

/** @file
 * This file includes declarations for all built-in jtag interfaces,
 * which are then listed in the adapter_drivers array.
 *
 * Dynamic loading can be implemented be searching for shared libraries
 * that contain an adapter_driver structure that can added to this list.
 */

#if BUILD_PARPORT == 1
extern struct adapter_driver parport_adapter_driver;
#endif
#if BUILD_DUMMY == 1
extern struct adapter_driver dummy_adapter_driver;
#endif
#if BUILD_FTDI == 1
extern struct adapter_driver ftdi_adapter_driver;
#endif
#if BUILD_USB_BLASTER == 1 || BUILD_USB_BLASTER_2 == 1
extern struct adapter_driver usb_blaster_adapter_driver;
#endif
#if BUILD_JTAG_VPI == 1
extern struct adapter_driver jtag_vpi_adapter_driver;
#endif
#if BUILD_VDEBUG == 1
extern struct adapter_driver vdebug_adapter_driver;
#endif
#if BUILD_JTAG_DPI == 1
extern struct adapter_driver jtag_dpi_adapter_driver;
#endif
#if BUILD_FT232R == 1
extern struct adapter_driver ft232r_adapter_driver;
#endif
#if BUILD_AMTJTAGACCEL == 1
extern struct adapter_driver amt_jtagaccel_adapter_driver;
#endif
#if BUILD_EP93XX == 1
extern struct adapter_driver ep93xx_adapter_driver;
#endif
#if BUILD_AT91RM9200 == 1
extern struct adapter_driver at91rm9200_adapter_driver;
#endif
#if BUILD_GW16012 == 1
extern struct adapter_driver gw16012_adapter_driver;
#endif
#if BUILD_PRESTO
extern struct adapter_driver presto_adapter_driver;
#endif
#if BUILD_USBPROG == 1
extern struct adapter_driver usbprog_adapter_driver;
#endif
#if BUILD_OPENJTAG == 1
extern struct adapter_driver openjtag_adapter_driver;
#endif
#if BUILD_JLINK == 1
extern struct adapter_driver jlink_adapter_driver;
#endif
#if BUILD_VSLLINK == 1
extern struct adapter_driver vsllink_adapter_driver;
#endif
#if BUILD_RLINK == 1
extern struct adapter_driver rlink_adapter_driver;
#endif
#if BUILD_ULINK == 1
extern struct adapter_driver ulink_adapter_driver;
#endif
#if BUILD_ARMJTAGEW == 1
extern struct adapter_driver armjtagew_adapter_driver;
#endif
#if BUILD_BUSPIRATE == 1
extern struct adapter_driver buspirate_adapter_driver;
#endif
#if BUILD_REMOTE_BITBANG == 1
extern struct adapter_driver remote_bitbang_adapter_driver;
#endif
#if BUILD_HLADAPTER == 1
extern struct adapter_driver hl_adapter_driver;
#endif
#if BUILD_OSBDM == 1
extern struct adapter_driver osbdm_adapter_driver;
#endif
#if BUILD_OPENDOUS == 1
extern struct adapter_driver opendous_adapter_driver;
#endif
#if BUILD_SYSFSGPIO == 1
extern struct adapter_driver sysfsgpio_adapter_driver;
#endif
#if BUILD_LINUXGPIOD == 1
extern struct adapter_driver linuxgpiod_adapter_driver;
#endif
#if BUILD_XLNX_PCIE_XVC == 1
extern struct adapter_driver xlnx_pcie_xvc_adapter_driver;
#endif
#if BUILD_AICE == 1
extern struct adapter_driver aice_adapter_driver;
#endif
#if BUILD_BCM2835GPIO == 1
extern struct adapter_driver bcm2835gpio_adapter_driver;
#endif
#if BUILD_CMSIS_DAP_USB == 1 || BUILD_CMSIS_DAP_HID == 1
extern struct adapter_driver cmsis_dap_adapter_driver;
#endif
#if BUILD_KITPROG == 1
extern struct adapter_driver kitprog_adapter_driver;
#endif
#if BUILD_IMX_GPIO == 1
extern struct adapter_driver imx_gpio_adapter_driver;
#endif
#if BUILD_XDS110 == 1
extern struct adapter_driver xds110_adapter_driver;
#endif
#if BUILD_HLADAPTER_STLINK == 1
extern struct adapter_driver stlink_dap_adapter_driver;
#endif
#if BUILD_RSHIM == 1
extern struct adapter_driver rshim_dap_adapter_driver;
#endif

/**
 * The list of built-in JTAG interfaces, containing entries for those
 * drivers that were enabled by the @c configure script.
 */
struct adapter_driver *adapter_drivers[] = {
#if BUILD_PARPORT == 1
		&parport_adapter_driver,
#endif
#if BUILD_DUMMY == 1
		&dummy_adapter_driver,
#endif
#if BUILD_FTDI == 1
		&ftdi_adapter_driver,
#endif
#if BUILD_USB_BLASTER || BUILD_USB_BLASTER_2 == 1
		&usb_blaster_adapter_driver,
#endif
#if BUILD_JTAG_VPI == 1
		&jtag_vpi_adapter_driver,
#endif
#if BUILD_VDEBUG == 1
		&vdebug_adapter_driver,
#endif
#if BUILD_JTAG_DPI == 1
		&jtag_dpi_adapter_driver,
#endif
#if BUILD_FT232R == 1
		&ft232r_adapter_driver,
#endif
#if BUILD_AMTJTAGACCEL == 1
		&amt_jtagaccel_adapter_driver,
#endif
#if BUILD_EP93XX == 1
		&ep93xx_adapter_driver,
#endif
#if BUILD_AT91RM9200 == 1
		&at91rm9200_adapter_driver,
#endif
#if BUILD_GW16012 == 1
		&gw16012_adapter_driver,
#endif
#if BUILD_PRESTO
		&presto_adapter_driver,
#endif
#if BUILD_USBPROG == 1
		&usbprog_adapter_driver,
#endif
#if BUILD_OPENJTAG == 1
		&openjtag_adapter_driver,
#endif
#if BUILD_JLINK == 1
		&jlink_adapter_driver,
#endif
#if BUILD_VSLLINK == 1
		&vsllink_adapter_driver,
#endif
#if BUILD_RLINK == 1
		&rlink_adapter_driver,
#endif
#if BUILD_ULINK == 1
		&ulink_adapter_driver,
#endif
#if BUILD_ARMJTAGEW == 1
		&armjtagew_adapter_driver,
#endif
#if BUILD_BUSPIRATE == 1
		&buspirate_adapter_driver,
#endif
#if BUILD_REMOTE_BITBANG == 1
		&remote_bitbang_adapter_driver,
#endif
#if BUILD_HLADAPTER == 1
		&hl_adapter_driver,
#endif
#if BUILD_OSBDM == 1
		&osbdm_adapter_driver,
#endif
#if BUILD_OPENDOUS == 1
		&opendous_adapter_driver,
#endif
#if BUILD_SYSFSGPIO == 1
		&sysfsgpio_adapter_driver,
#endif
#if BUILD_LINUXGPIOD == 1
		&linuxgpiod_adapter_driver,
#endif
#if BUILD_XLNX_PCIE_XVC == 1
		&xlnx_pcie_xvc_adapter_driver,
#endif
#if BUILD_AICE == 1
		&aice_adapter_driver,
#endif
#if BUILD_BCM2835GPIO == 1
		&bcm2835gpio_adapter_driver,
#endif
#if BUILD_CMSIS_DAP_USB == 1 || BUILD_CMSIS_DAP_HID == 1
		&cmsis_dap_adapter_driver,
#endif
#if BUILD_KITPROG == 1
		&kitprog_adapter_driver,
#endif
#if BUILD_IMX_GPIO == 1
		&imx_gpio_adapter_driver,
#endif
#if BUILD_XDS110 == 1
		&xds110_adapter_driver,
#endif
#if BUILD_HLADAPTER_STLINK == 1
		&stlink_dap_adapter_driver,
#endif
#if BUILD_RSHIM == 1
		&rshim_dap_adapter_driver,
#endif
		NULL,
	};
