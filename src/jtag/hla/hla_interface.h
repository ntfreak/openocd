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

#ifndef OPENOCD_JTAG_HLA_HLA_INTERFACE_H
#define OPENOCD_JTAG_HLA_HLA_INTERFACE_H

/** */
struct target;
/** */
enum e_hl_transports;
/** */
extern const char *hl_transports[];

#define HLA_MAX_USB_IDS 8

struct hl_interface_param_s {
	/** */
	const char *device_desc;
	/** */
	const char *serial;
	/** List of recognised VIDs */
	uint16_t vid[HLA_MAX_USB_IDS + 1];
	/** List of recognised PIDs */
	uint16_t pid[HLA_MAX_USB_IDS + 1];
	/** */
	enum hl_transports transport;
	/** */
	bool connect_under_reset;
	/** Initial interface clock clock speed */
	int initial_interface_speed;
};

struct hl_interface_s {
	/** */
	struct hl_interface_param_s param;
	/** */
	const struct hl_layout *layout;
	/** */
	void *handle;
};

/** */
int hl_interface_open(enum hl_transports tr);
/** */

int hl_interface_init_target(struct target *t);
int hl_interface_init_reset(void);
int hl_interface_override_target(const char **targetname);

#endif /* OPENOCD_JTAG_HLA_HLA_INTERFACE_H */
