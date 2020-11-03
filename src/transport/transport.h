/*
 * Copyright (c) 2010 by David Brownell
 * Copyright (C) 2011 Tomasz Boleslaw CEDRO (http://www.tomek.cedro.info)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OPENOCD_TRANSPORT_TRANSPORT_H
#define OPENOCD_TRANSPORT_TRANSPORT_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "helper/command.h"

/**
 * Wrapper for transport lifecycle operations.
 *
 * OpenOCD talks to targets through some kind of debugging
 * or programming adapter, using some protocol that probably
 * has target-specific aspects.
 *
 * A "transport" reflects electrical protocol to the target,
 * e..g jtag, swd, spi, uart, ... NOT the messaging protocols
 * layered over it (e.g. JTAG has eICE, CoreSight, Nexus, OnCE,
 * and more).
 *
 * In addition to the lifecycle operations packaged by this
 * structure, a transport also involves  an interface supported
 * by debug adapters and used by components such as debug targets.
 * For non-debug transports,  there may be interfaces used to
 * write to flash chips.
 */
struct transport {
	/**
	 * Each transport has a unique name, used to select it
	 * from among the alternatives.  Examples might include
	 * "jtag", * "swd", "AVR_ISP" and more.
	 */
	const char *name;

	/**
	 * When a transport is selected, this method registers
	 * its commands and activates the transport (e.g. resets
	 * the link).
	 *
	 * After those commands are registered, they will often
	 * be used for further configuration of the debug link.
	 */
	int (*select)(struct command_context *ctx);

	/**
	 * server startup uses this method to validate transport
	 * configuration.  (For example, with JTAG this interrogates
	 * the scan chain against the list of expected TAPs.)
	 */
	int (*init)(struct command_context *ctx);

	/**
	 * Optional. If defined, allows transport to override target
	 * name prior to initialisation.
	 *
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*override_target)(const char **targetname);

	/**
	 * Transports are stored in a singly linked list.
	 */
	struct transport *next;
};

int transport_register(struct transport *new_transport);

struct transport *get_current_transport(void);

int transport_register_commands(struct command_context *ctx);

COMMAND_HELPER(transport_list_parse, char ***vector);

int allow_transports(struct command_context *ctx, const char * const *vector);

bool transport_is_jtag(void);
bool transport_is_swd(void);
bool transport_is_dapdirect_jtag(void);
bool transport_is_dapdirect_swd(void);
bool transport_is_swim(void);

#if BUILD_HLADAPTER
bool transport_is_hla(void);
#else
static inline bool transport_is_hla(void)
{
	return false;
}
#endif

#endif /* OPENOCD_TRANSPORT_TRANSPORT_H */
