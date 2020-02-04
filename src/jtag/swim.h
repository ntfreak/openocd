/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2020 by Antonio Borneo <borneo.antonio@gmail.com
 */

/**
 * @file
 * This file implements support for STMicroelectronics debug protocol SWIM
 * (Single Wire Interface Module).
 */

#ifndef OPENOCD_JTAG_SWIM_H
#define OPENOCD_JTAG_SWIM_H

#define SWIM_FREQ_LOW   363
#define SWIM_FREQ_HIGH  800

struct swim_driver {
	/**
	 * Send SRST (system reset) command to target.
	 *
	 * @return ERROR_OK on success, else a fault code.
	 */
	int (*srst)(void);

	/**
	 * Read target memory through ROTF (read on-the-fly) command.
	 *
	 * @param addr Start address to read data from target memory.
	 * @param size Size in bytes of data units, 1, 2 or 4.
	 * @param count Number of units (size units, not bytes) to read.
	 * @param buffer Data buffer to receive data.
	 * @return ERROR_OK on success, else a fault code.
	 */
	int (*read_mem)(uint32_t addr, uint32_t size, uint32_t count,
					uint8_t *buffer);

	/**
	 * Write target memory through WOTF (write on-the-fly) command.
	 *
	 * @param addr Start address to write data to target memory.
	 * @param size Size in bytes of data units, 1, 2 or 4.
	 * @param count Number of units (size units, not bytes) to write.
	 * @param buffer Data buffer to write.
	 * @return ERROR_OK on success, else a fault code.
	 */
	int (*write_mem)(uint32_t addr, uint32_t size, uint32_t count,
					 const uint8_t *buffer);

	/**
	 * Reconnect to the target.
	 * Should be reworked to be more generic and not linked to current
	 * implementation in stlink driver.
	 *
	 * @return ERROR_OK on success, else a fault code.
	 */
	int (*reconnect)(void);
};

int swim_system_reset(void);
int swim_read_mem(uint32_t addr, uint32_t size, uint32_t count,
				  uint8_t *buffer);
int swim_write_mem(uint32_t addr, uint32_t size, uint32_t count,
				   const uint8_t *buffer);
int swim_reconnect(void);

#endif /* OPENOCD_JTAG_SWIM_H */
