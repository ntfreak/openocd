/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_ARM_TPIU_SWO_H
#define OPENOCD_TARGET_ARM_TPIU_SWO_H

/* Values should match TPIU_SPPR_PROTOCOL_xxx */
enum tpiu_pin_protocol {
	TPIU_PIN_PROTOCOL_SYNC = 0,                 /**< synchronous trace output */
	TPIU_PIN_PROTOCOL_ASYNC_MANCHESTER = 1,     /**< asynchronous output with Manchester coding */
	TPIU_PIN_PROTOCOL_ASYNC_UART = 2,           /**< asynchronous output with NRZ coding */
};

/* START_DEPRECATED_TPIU */
/* DEPRECATED: emulation of old command 'tpiu config' */
extern const struct command_registration arm_tpiu_deprecated_command_handlers[];
/* END_DEPRECATED_TPIU */

int arm_tpiu_swo_register_commands(struct command_context *cmd_ctx);
int arm_tpiu_swo_cleanup_all(void);

#endif /* OPENOCD_TARGET_ARM_TPIU_SWO_H */
