/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_ARM_TPIU_SWO_H
#define OPENOCD_TARGET_ARM_TPIU_SWO_H

int arm_tpiu_swo_register_commands(struct command_context *cmd_ctx);
int arm_tpiu_swo_cleanup_all(void);

#endif /* OPENOCD_TARGET_ARM_TPIU_SWO_H */
