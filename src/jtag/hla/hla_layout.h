/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#ifndef OPENOCD_JTAG_HLA_HLA_LAYOUT_H
#define OPENOCD_JTAG_HLA_HLA_LAYOUT_H

#include <target/armv7m_trace.h>

/** */
struct hl_interface_s;
struct hl_interface_param_s;

/** */
extern struct hl_layout_api_s stlink_usb_layout_api;
extern struct hl_layout_api_s icdi_usb_layout_api;
extern struct hl_layout_api_s nulink_usb_layout_api;

/** */
struct hl_layout_api_s {
	/** */
	int (*open)(struct hl_interface_param_s *param, void **handle);
	/** */
	int (*close)(void *handle);
	/** */
	int (*reset)(void *handle);
	/** */
	int (*assert_srst)(void *handle, int srst);
	/** */
	int (*run)(void *handle);
	/** */
	int (*halt)(void *handle);
	/** */
	int (*step)(void *handle);
	/** */
	int (*read_regs)(void *handle);
	/**
	 * Read one register from the target
	 *
	 * @param handle A pointer to the device-specific handle
	 * @param regsel Register selection index compatible with all the
	 * values allowed by armv7m DCRSR.REGSEL
	 * @param val A pointer to retrieve the register value
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*read_reg)(void *handle, unsigned int regsel, uint32_t *val);
	/**
	 * Write one register to the target
	 * @param handle A pointer to the device-specific handle
	 * @param regsel Register selection index compatible with all the
	 * values allowed by armv7m DCRSR.REGSEL
	 * @param val The value to be written in the register
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*write_reg)(void *handle, unsigned int regsel, uint32_t val);
	/** */
	int (*read_mem)(void *handle, uint32_t addr, uint32_t size,
			uint32_t count, uint8_t *buffer);
	/** */
	int (*write_mem)(void *handle, uint32_t addr, uint32_t size,
			uint32_t count, const uint8_t *buffer);
	/** */
	int (*write_debug_reg)(void *handle, uint32_t addr, uint32_t val);
	/**
	 * Read the idcode of the target connected to the adapter
	 *
	 * If the adapter doesn't support idcode retrieval, this callback should
	 * store 0 to indicate a wildcard match.
	 *
	 * @param handle A pointer to the device-specific handle
	 * @param idcode Storage for the detected idcode
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*idcode)(void *handle, uint32_t *idcode);
	/** */
	int (*override_target)(const char *targetname);
	/** */
	int (*custom_command)(void *handle, const char *command);
	/** */
	int (*speed)(void *handle, int khz, bool query);
	/**
	 * Configure trace parameters for the adapter
	 *
	 * @param handle A handle to adapter
	 * @param enabled Whether to enable trace
	 * @param pin_protocol Configured pin protocol
	 * @param port_size Trace port width for sync mode
	 * @param trace_freq A pointer to the configured trace
	 * frequency; if it points to 0, the adapter driver must write
	 * its maximum supported rate there
	 * @returns ERROR_OK on success, an error code on failure.
	 */
	int (*config_trace)(void *handle, bool enabled,
				enum tpiu_pin_protocol pin_protocol, uint32_t port_size,
				unsigned int *trace_freq, unsigned int traceclkin_freq,
				uint16_t *prescaler);
	/**
	 * Poll for new trace data
	 *
	 * @param handle A handle to adapter
	 * @param buf A pointer to buffer to store received data
	 * @param size A pointer to buffer size; must be filled with
	 * the actual amount of bytes written
	 *
	 * @returns ERROR_OK on success, an error code on failure.
	 */
	int (*poll_trace)(void *handle, uint8_t *buf, size_t *size);
	/** */
	enum target_state (*state)(void *fd);
};

/** */
struct hl_layout {
	/** */
	char *name;
	/** */
	int (*open)(struct hl_interface_s *adapter);
	/** */
	int (*close)(struct hl_interface_s *adapter);
	/** */
	struct hl_layout_api_s *api;
};

/** */
const struct hl_layout *hl_layout_get_list(void);
/** */
int hl_layout_init(struct hl_interface_s *adapter);

#endif /* OPENOCD_JTAG_HLA_HLA_LAYOUT_H */
