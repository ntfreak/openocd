#ifndef OPENOCD_JTAG_DRIVERS_CMSIS_DAP_H
#define OPENOCD_JTAG_DRIVERS_CMSIS_DAP_H

#include <stdint.h>

struct cmsis_dap_backend;
struct cmsis_dap_backend_data;
struct command_registration;

struct cmsis_dap {
	struct cmsis_dap_backend_data *bdata;
	const struct cmsis_dap_backend *backend;
	uint16_t packet_size;
	int packet_count;
	uint8_t *packet_buffer;
	uint8_t caps;
	uint8_t mode;
};

struct cmsis_dap_backend {
	const char *name;
	int (*open)(struct cmsis_dap *dap, uint16_t vids[], uint16_t pids[], char *serial);
	void (*close)(struct cmsis_dap *dap);
	int (*read)(struct cmsis_dap *dap, int timeout_ms);
	int (*write)(struct cmsis_dap *dap, int len, int timeout_ms);
};

extern const struct cmsis_dap_backend cmsis_dap_hid_backend;
extern const struct cmsis_dap_backend cmsis_dap_usb_backend;
extern const struct command_registration cmsis_dap_usb_subcommand_handlers[];

#endif
