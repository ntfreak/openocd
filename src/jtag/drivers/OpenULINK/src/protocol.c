/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
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

#include "protocol.h"
#include "jtag.h"
#include "delay.h"
#include "usb.h"
#include "io.h"
#include "msgtypes.h"

#include "reg_ezusb.h"

/**
 * @file
 * Implementation of the OpenULINK communication protocol.
 *
 * The OpenULINK protocol uses one OUT and one IN endpoint. These two endpoints
 * are configured to use the maximum packet size for full-speed transfers,
 * 64 bytes. Commands always start with a command ID (see msgtypes.h for
 * command ID definitions) and contain zero or more payload data bytes in both
 * transfer directions (IN and OUT). The payload
 *
 * Almost all commands contain a fixed number of payload data bytes. The number
 * of payload data bytes for the IN and OUT direction does not need to be the
 * same.
 *
 * Multiple commands may be sent in one EP2 Bulk-OUT packet. Because the
 * OpenULINK firmware does not perform bounds checking for EP2 Bulk-IN packets,
 * the host MUST ensure that the commands sent in the OUT packet require a
 * maximum of 64 bytes of IN data.
 */

/** Index in EP2 Bulk-OUT data buffer that contains the current command ID */
volatile uint8_t cmd_id_index;

/** Number of data bytes already in EP2 Bulk-IN buffer */
volatile uint8_t payload_index_in;

/**
 * Execute a SET_LEDS command.
 */
void execute_set_led_command(void)
{
	uint8_t led_state = OUT2BUF[cmd_id_index + 1];

	if (led_state & RUN_LED_ON)
		SET_RUN_LED();

	if (led_state & COM_LED_ON)
		SET_COM_LED();

	if (led_state & RUN_LED_OFF)
		CLEAR_RUN_LED();

	if (led_state & COM_LED_OFF)
		CLEAR_COM_LED();
}

/**
 * Executes one command and updates global command indexes.
 *
 * @return true if this command was the last command.
 * @return false if there are more commands within the current contents of the
 * Bulk EP2-OUT data buffer.
 */
bool execute_command(void)
{
	uint8_t usb_out_bytecount, usb_in_bytecount;
	uint16_t signal_state;
	uint16_t count;

	/* Most commands do not transfer IN data. To save code space, we write 0 to
	 * usb_in_bytecount here, then modify it in the switch statement below where
	 * necessary */
	usb_in_bytecount = 0;

	switch (OUT2BUF[cmd_id_index] /* Command ID */) {
		case CMD_SCAN_IN:
			usb_out_bytecount = 5;
			usb_in_bytecount = OUT2BUF[cmd_id_index + 1];
			jtag_scan_in(cmd_id_index + 1, payload_index_in);
			break;
		case CMD_SCAN_OUT:
			usb_out_bytecount = OUT2BUF[cmd_id_index + 1] + 5;
			jtag_scan_out(cmd_id_index + 1);
			break;
		case CMD_SCAN_IO:
			usb_in_bytecount = OUT2BUF[cmd_id_index + 1];
			usb_out_bytecount = usb_in_bytecount + 5;
			jtag_scan_io(cmd_id_index + 1, payload_index_in);
			break;
		case CMD_CLOCK_TMS:
			usb_out_bytecount = 2;
			jtag_clock_tms(OUT2BUF[cmd_id_index + 1], OUT2BUF[cmd_id_index + 2]);
			break;
		case CMD_CLOCK_TCK:
			usb_out_bytecount = 2;
			count = (uint16_t)OUT2BUF[cmd_id_index + 1];
			count |= ((uint16_t)OUT2BUF[cmd_id_index + 2]) << 8;
			jtag_clock_tck(count);
			break;
		case CMD_SLOW_SCAN_IN:
			usb_out_bytecount = 5;
			usb_in_bytecount = OUT2BUF[cmd_id_index + 1];
			jtag_slow_scan_in(cmd_id_index + 1, payload_index_in);
			break;
		case CMD_SLOW_SCAN_OUT:
			usb_out_bytecount = OUT2BUF[cmd_id_index + 1] + 5;
			jtag_slow_scan_out(cmd_id_index + 1);
			break;
		case CMD_SLOW_SCAN_IO:
			usb_in_bytecount = OUT2BUF[cmd_id_index + 1];
			usb_out_bytecount = usb_in_bytecount + 5;
			jtag_slow_scan_io(cmd_id_index + 1, payload_index_in);
			break;
		case CMD_SLOW_CLOCK_TMS:
			usb_out_bytecount = 2;
			jtag_slow_clock_tms(OUT2BUF[cmd_id_index + 1], OUT2BUF[cmd_id_index + 2]);
			break;
		case CMD_SLOW_CLOCK_TCK:
			usb_out_bytecount = 2;
			count = (uint16_t)OUT2BUF[cmd_id_index + 1];
			count |= ((uint16_t)OUT2BUF[cmd_id_index + 2]) << 8;
			jtag_slow_clock_tck(count);
			break;
		case CMD_SLEEP_US:
			usb_out_bytecount = 2;
			count = (uint16_t)OUT2BUF[cmd_id_index + 1];
			count |= ((uint16_t)OUT2BUF[cmd_id_index + 2]) << 8;
			delay_us(count);
			break;
		case CMD_SLEEP_MS:
			usb_out_bytecount = 2;
			count = (uint16_t)OUT2BUF[cmd_id_index + 1];
			count |= ((uint16_t)OUT2BUF[cmd_id_index + 2]) << 8;
			delay_ms(count);
			break;
		case CMD_GET_SIGNALS:
			usb_out_bytecount = 0;
			usb_in_bytecount = 2;
			signal_state = jtag_get_signals();
			IN2BUF[payload_index_in] = (signal_state >> 8) & 0x00FF;
			IN2BUF[payload_index_in + 1] = signal_state & 0x00FF;
			break;
		case CMD_SET_SIGNALS:
			usb_out_bytecount = 2;
			jtag_set_signals(OUT2BUF[cmd_id_index + 1], OUT2BUF[cmd_id_index + 2]);
			break;
		case CMD_CONFIGURE_TCK_FREQ:
			usb_out_bytecount = 5;
			jtag_configure_tck_delay(
			OUT2BUF[cmd_id_index + 1],	/* scan_in */
			OUT2BUF[cmd_id_index + 2],	/* scan_out */
			OUT2BUF[cmd_id_index + 3],	/* scan_io */
			OUT2BUF[cmd_id_index + 4],	/* clock_tck */
			OUT2BUF[cmd_id_index + 5]);	/* clock_tms */
			break;
		case CMD_SET_LEDS:
			usb_out_bytecount = 1;
			execute_set_led_command();
			break;
		case CMD_TEST:
			usb_out_bytecount = 1;
			/* Do nothing... This command is only used to test if the device is ready
			 * to accept new commands */
			break;
		default:
			/* Should never be reached */
			usb_out_bytecount = 0;
			break;
	}

	/* Update EP2 Bulk-IN data byte count */
	payload_index_in += usb_in_bytecount;

	/* Determine if this was the last command */
	if ((cmd_id_index + usb_out_bytecount + 1) >= OUT2BC)
		return true;
	else {
		/* Not the last command, update cmd_id_index */
		cmd_id_index += (usb_out_bytecount + 1);
		return false;
	}
}

/**
 * Forever wait for commands and execute them as they arrive.
 */
void command_loop(void)
{
	bool last_command;

	while (1) {
		cmd_id_index = 0;
		payload_index_in = 0;

		/* Wait until host sends EP2 Bulk-OUT packet */
		while (!EP2_out)
			;
		EP2_out = 0;

		/* Turn on COM LED to indicate command execution */
		SET_COM_LED();

		/* Execute the commands */
		last_command = false;
		while (last_command == false)
			last_command = execute_command();

		CLEAR_COM_LED();

		/* Send back EP2 Bulk-IN packet if required */
		if (payload_index_in > 0) {
			IN2BC = payload_index_in;
			while (!EP2_in)
				;
			EP2_in = 0;
		}

		/* Re-arm EP2-OUT after command execution */
		OUT2BC = 0;
	}
}
