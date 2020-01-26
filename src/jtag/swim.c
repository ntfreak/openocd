/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2020 by Antonio Borneo <borneo.antonio@gmail.com
 *
 * SWIM (Single Wire Interface Module) is a low-pin-count debug protocol
 * used by STMicroelectronics MCU family STM8 and documented in UM470
 * https://www.st.com/resource/en/user_manual/cd00173911.pdf
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "interface.h"
#include "swim.h"
#include "jtag/hla/hla_transport.h"
#include "jtag/hla/hla_interface.h"
#include "jtag/hla/hla_layout.h"

extern struct adapter_driver *adapter_driver;

int swim_system_reset(void)
{
	assert(adapter_driver->hla_if);

	return adapter_driver->hla_if->layout->api->reset(adapter_driver->hla_if->handle);
}

int swim_read_mem(uint32_t addr, uint32_t size, uint32_t count,
				  uint8_t *buffer)
{
	assert(adapter_driver->hla_if);

	return adapter_driver->hla_if->layout->api->read_mem(adapter_driver->hla_if->handle, addr, size, count, buffer);
}

int swim_write_mem(uint32_t addr, uint32_t size, uint32_t count,
				   const uint8_t *buffer)
{
	assert(adapter_driver->hla_if);

	return adapter_driver->hla_if->layout->api->write_mem(adapter_driver->hla_if->handle, addr, size, count, buffer);
}

int swim_reconnect(void)
{
	assert(adapter_driver->hla_if);

	return adapter_driver->hla_if->layout->api->state(adapter_driver->hla_if->handle);
}
