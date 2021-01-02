/***************************************************************************
 *   Copyright (C) 2011-2013 by Martin Schmoelzer                          *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <target/image.h>
#include <libusb.h>
#include "libusb_helper.h"
#include "OpenULINK/include/msgtypes.h"

/** USB Vendor ID of ULINK device in unconfigured state (no firmware loaded
 *  yet) or with OpenULINK firmware. */
#define ULINK_VID                0xC251

/** USB Product ID of ULINK device in unconfigured state (no firmware loaded
 *  yet) or with OpenULINK firmware. */
#define ULINK_PID                0x2710

/** Address of EZ-USB CPU Control & Status register. This register can be
 *  written by issuing a Control EP0 vendor request. */
#define CPUCS_REG                0x7F92

/** USB Control EP0 bRequest: "Firmware Load". */
#define REQUEST_FIRMWARE_LOAD    0xA0

/** Value to write into CPUCS to put EZ-USB into reset. */
#define CPU_RESET                0x01

/** Value to write into CPUCS to put EZ-USB out of reset. */
#define CPU_START                0x00

/** Base address of firmware in EZ-USB code space. */
#define FIRMWARE_ADDR            0x0000

/** USB interface number */
#define USB_INTERFACE            0

/** libusb timeout in ms */
#define USB_TIMEOUT              5000

/** Delay (in microseconds) to wait while EZ-USB performs ReNumeration. */
#define ULINK_RENUMERATION_DELAY 1500000

/** Default location of OpenULINK firmware image. */
#define ULINK_FIRMWARE_FILE      PKGDATADIR "/OpenULINK/ulink_firmware.hex"

/** Maximum size of a single firmware section. Entire EZ-USB code space = 8kB */
#define SECTION_BUFFERSIZE       8192

/** Tuning of OpenOCD SCAN commands split into multiple OpenULINK commands. */
#define SPLIT_SCAN_THRESHOLD     10

/** ULINK hardware type */
enum ulink_type {
	/** Original ULINK adapter, based on Cypress EZ-USB (AN2131):
	 *  Full JTAG support, no SWD support. */
	ULINK_1,

	/** Newer ULINK adapter, based on NXP LPC2148. Currently unsupported. */
	ULINK_2,

	/** Newer ULINK adapter, based on EZ-USB FX2 + FPGA. Currently unsupported. */
	ULINK_PRO,

	/** Newer ULINK adapter, possibly based on ULINK 2. Currently unsupported. */
	ULINK_ME
};

enum ulink_payload_direction {
	PAYLOAD_DIRECTION_OUT,
	PAYLOAD_DIRECTION_IN
};

enum ulink_delay_type {
	DELAY_CLOCK_TCK,
	DELAY_CLOCK_TMS,
	DELAY_SCAN_IN,
	DELAY_SCAN_OUT,
	DELAY_SCAN_IO
};

/**
 * OpenULINK command (OpenULINK command queue element).
 *
 * For the OUT direction payload, things are quite easy: Payload is stored
 * in a rather small array (up to 63 bytes), the payload is always allocated
 * by the function generating the command and freed by ulink_clear_queue().
 *
 * For the IN direction payload, things get a little bit more complicated:
 * The maximum IN payload size for a single command is 64 bytes. Assume that
 * a single OpenOCD command needs to scan 256 bytes. This results in the
 * generation of four OpenULINK commands. The function generating these
 * commands shall allocate an uint8_t[256] array. Each command's #payload_in
 * pointer shall point to the corresponding offset where IN data shall be
 * placed, while #payload_in_start shall point to the first element of the 256
 * byte array.
 * - first command:  #payload_in_start + 0
 * - second command: #payload_in_start + 64
 * - third command:  #payload_in_start + 128
 * - fourth command: #payload_in_start + 192
 *
 * The last command sets #needs_postprocessing to true.
 */
struct ulink_cmd {
	uint8_t id;			/**< ULINK command ID */

	uint8_t *payload_out;		/**< OUT direction payload data */
	uint8_t payload_out_size;	/**< OUT direction payload size for this command */

	uint8_t *payload_in_start;	/**< Pointer to first element of IN payload array */
	uint8_t *payload_in;		/**< Pointer where IN payload shall be stored */
	uint8_t payload_in_size;	/**< IN direction payload size for this command */

	/** Indicates if this command needs post-processing */
	bool needs_postprocessing;

	/** Indicates if ulink_clear_queue() should free payload_in_start  */
	bool free_payload_in_start;

	/** Pointer to corresponding OpenOCD command for post-processing */
	struct jtag_command *cmd_origin;

	struct ulink_cmd *next;		/**< Pointer to next command (linked list) */
};

/** Describes one driver instance */
struct ulink {
	struct libusb_context *libusb_ctx;
	struct libusb_device_handle *usb_device_handle;
	enum ulink_type type;

	unsigned int ep_in;		/**< IN endpoint number */
	unsigned int ep_out;		/**< OUT endpoint number */

	int delay_scan_in;	/**< Delay value for SCAN_IN commands */
	int delay_scan_out;	/**< Delay value for SCAN_OUT commands */
	int delay_scan_io;	/**< Delay value for SCAN_IO commands */
	int delay_clock_tck;	/**< Delay value for CLOCK_TMS commands */
	int delay_clock_tms;	/**< Delay value for CLOCK_TCK commands */

	int commands_in_queue;		/**< Number of commands in queue */
	struct ulink_cmd *queue_start;	/**< Pointer to first command in queue */
	struct ulink_cmd *queue_end;	/**< Pointer to last command in queue */
};

/**************************** Function Prototypes *****************************/

/* USB helper functions */
static int ulink_usb_open(struct ulink **device);
static int ulink_usb_close(struct ulink **device);

/* ULINK MCU (Cypress EZ-USB) specific functions */
static int ulink_cpu_reset(struct ulink *device, unsigned char reset_bit);
static int ulink_load_firmware_and_renumerate(struct ulink **device, const char *filename,
		uint32_t delay);
static int ulink_load_firmware(struct ulink *device, const char *filename);
static int ulink_write_firmware_section(struct ulink *device,
		struct image *firmware_image, int section_index);

/* Generic helper functions */
static void ulink_print_signal_states(uint8_t input_signals, uint8_t output_signals);

/* OpenULINK command generation helper functions */
static int ulink_allocate_payload(struct ulink_cmd *ulink_cmd, int size,
		enum ulink_payload_direction direction);

/* OpenULINK command queue helper functions */
static int ulink_get_queue_size(struct ulink *device,
		enum ulink_payload_direction direction);
static void ulink_clear_queue(struct ulink *device);
static int ulink_append_queue(struct ulink *device, struct ulink_cmd *ulink_cmd);
static int ulink_execute_queued_commands(struct ulink *device, int timeout);

static void ulink_print_queue(struct ulink *device);

static int ulink_append_scan_cmd(struct ulink *device,
		enum scan_type scan_type,
		int scan_size_bits,
		uint8_t *tdi,
		uint8_t *tdo_start,
		uint8_t *tdo,
		uint8_t tms_count_start,
		uint8_t tms_sequence_start,
		uint8_t tms_count_end,
		uint8_t tms_sequence_end,
		struct jtag_command *origin,
		bool postprocess);
static int ulink_append_clock_tms_cmd(struct ulink *device, uint8_t count,
		uint8_t sequence);
static int ulink_append_clock_tck_cmd(struct ulink *device, uint16_t count);
static int ulink_append_get_signals_cmd(struct ulink *device);
static int ulink_append_set_signals_cmd(struct ulink *device, uint8_t low,
		uint8_t high);
static int ulink_append_sleep_cmd(struct ulink *device, uint32_t us);
static int ulink_append_configure_tck_cmd(struct ulink *device,
		int delay_scan_in,
		int delay_scan_out,
		int delay_scan_io,
		int delay_tck,
		int delay_tms);
static int __attribute__((unused)) ulink_append_led_cmd(struct ulink *device, uint8_t led_state);
static int ulink_append_test_cmd(struct ulink *device);

/* OpenULINK TCK frequency helper functions */
static int ulink_calculate_delay(enum ulink_delay_type type, long f, int *delay);

/* Interface between OpenULINK and OpenOCD */
static void ulink_set_end_state(tap_state_t endstate);
static int ulink_queue_statemove(struct ulink *device);

static int ulink_queue_scan(struct ulink *device, struct jtag_command *cmd);
static int ulink_queue_tlr_reset(struct ulink *device, struct jtag_command *cmd);
static int ulink_queue_runtest(struct ulink *device, struct jtag_command *cmd);
static int ulink_queue_reset(struct ulink *device, struct jtag_command *cmd);
static int ulink_queue_pathmove(struct ulink *device, struct jtag_command *cmd);
static int ulink_queue_sleep(struct ulink *device, struct jtag_command *cmd);
static int ulink_queue_stableclocks(struct ulink *device, struct jtag_command *cmd);

static int ulink_post_process_scan(struct ulink_cmd *ulink_cmd);
static int ulink_post_process_queue(struct ulink *device);

/* adapter driver functions */
static int ulink_execute_queue(void);
static int ulink_khz(int khz, int *jtag_speed);
static int ulink_speed(int speed);
static int ulink_speed_div(int speed, int *khz);
static int ulink_init(void);
static int ulink_quit(void);

/****************************** Global Variables ******************************/

static struct ulink *ulink_handle;

/**************************** USB helper functions ****************************/

/**
 * Opens the ULINK device
 *
 * Currently, only the original ULINK is supported
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_usb_open(struct ulink **device)
{
	ssize_t num_devices, i;
	bool found;
	libusb_device **usb_devices;
	struct libusb_device_descriptor usb_desc;
	struct libusb_device_handle *usb_device_handle;

	num_devices = libusb_get_device_list((*device)->libusb_ctx, &usb_devices);

	if (num_devices <= 0)
		return ERROR_FAIL;

	found = false;
	for (i = 0; i < num_devices; i++) {
		if (libusb_get_device_descriptor(usb_devices[i], &usb_desc) != 0)
			continue;
		else if (usb_desc.idVendor == ULINK_VID && usb_desc.idProduct == ULINK_PID) {
			found = true;
			break;
		}
	}

	if (!found)
		return ERROR_FAIL;

	if (libusb_open(usb_devices[i], &usb_device_handle) != 0)
		return ERROR_FAIL;
	libusb_free_device_list(usb_devices, 1);

	(*device)->usb_device_handle = usb_device_handle;
	(*device)->type = ULINK_1;

	return ERROR_OK;
}

/**
 * Releases the ULINK interface and closes the USB device handle.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_usb_close(struct ulink **device)
{
	if (libusb_release_interface((*device)->usb_device_handle, 0) != 0)
		return ERROR_FAIL;

	libusb_close((*device)->usb_device_handle);

	(*device)->usb_device_handle = NULL;

	return ERROR_OK;
}

/******************* ULINK CPU (EZ-USB) specific functions ********************/

/**
 * Writes '0' or '1' to the CPUCS register, putting the EZ-USB CPU into reset
 * or out of reset.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param reset_bit 0 to put CPU into reset, 1 to put CPU out of reset.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_cpu_reset(struct ulink *device, unsigned char reset_bit)
{
	int ret;

	ret = libusb_control_transfer(device->usb_device_handle,
			(LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE),
			REQUEST_FIRMWARE_LOAD, CPUCS_REG, 0, &reset_bit, 1, USB_TIMEOUT);

	/* usb_control_msg() returns the number of bytes transferred during the
	 * DATA stage of the control transfer - must be exactly 1 in this case! */
	if (ret != 1)
		return ERROR_FAIL;
	return ERROR_OK;
}

/**
 * Puts the ULINK's EZ-USB microcontroller into reset state, downloads
 * the firmware image, resumes the microcontroller and re-enumerates
 * USB devices.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 *  The usb_handle member will be modified during re-enumeration.
 * @param filename path to the Intel HEX file containing the firmware image.
 * @param delay the delay to wait for the device to re-enumerate.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_load_firmware_and_renumerate(struct ulink **device,
	const char *filename, uint32_t delay)
{
	int ret;

	/* Basic process: After downloading the firmware, the ULINK will disconnect
	 * itself and re-connect after a short amount of time so we have to close
	 * the handle and re-enumerate USB devices */

	ret = ulink_load_firmware(*device, filename);
	if (ret != ERROR_OK)
		return ret;

	ret = ulink_usb_close(device);
	if (ret != ERROR_OK)
		return ret;

	usleep(delay);

	ret = ulink_usb_open(device);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

/**
 * Downloads a firmware image to the ULINK's EZ-USB microcontroller
 * over the USB bus.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param filename an absolute or relative path to the Intel HEX file
 *  containing the firmware image.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_load_firmware(struct ulink *device, const char *filename)
{
	struct image ulink_firmware_image;
	int ret;

	ret = ulink_cpu_reset(device, CPU_RESET);
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not halt ULINK CPU");
		return ret;
	}

	ulink_firmware_image.base_address = 0;
	ulink_firmware_image.base_address_set = false;

	ret = image_open(&ulink_firmware_image, filename, "ihex");
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not load firmware image");
		return ret;
	}

	/* Download all sections in the image to ULINK */
	for (unsigned int i = 0; i < ulink_firmware_image.num_sections; i++) {
		ret = ulink_write_firmware_section(device, &ulink_firmware_image, i);
		if (ret != ERROR_OK)
			return ret;
	}

	image_close(&ulink_firmware_image);

	ret = ulink_cpu_reset(device, CPU_START);
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not restart ULINK CPU");
		return ret;
	}

	return ERROR_OK;
}

/**
 * Send one contiguous firmware section to the ULINK's EZ-USB microcontroller
 * over the USB bus.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param firmware_image pointer to the firmware image that contains the section
 *  which should be sent to the ULINK's EZ-USB microcontroller.
 * @param section_index index of the section within the firmware image.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_write_firmware_section(struct ulink *device,
	struct image *firmware_image, int section_index)
{
	uint16_t addr, size, bytes_remaining, chunk_size;
	uint8_t data[SECTION_BUFFERSIZE];
	uint8_t *data_ptr = data;
	size_t size_read;
	int ret;

	size = (uint16_t)firmware_image->sections[section_index].size;
	addr = (uint16_t)firmware_image->sections[section_index].base_address;

	LOG_DEBUG("section %02i at addr 0x%04x (size 0x%04x)", section_index, addr,
		size);

	/* Copy section contents to local buffer */
	ret = image_read_section(firmware_image, section_index, 0, size, data,
			&size_read);

	if ((ret != ERROR_OK) || (size_read != size)) {
		/* Propagating the return code would return '0' (misleadingly indicating
		 * successful execution of the function) if only the size check fails. */
		return ERROR_FAIL;
	}

	bytes_remaining = size;

	/* Send section data in chunks of up to 64 bytes to ULINK */
	while (bytes_remaining > 0) {
		if (bytes_remaining > 64)
			chunk_size = 64;
		else
			chunk_size = bytes_remaining;

		ret = libusb_control_transfer(device->usb_device_handle,
				(LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE),
				REQUEST_FIRMWARE_LOAD, addr, FIRMWARE_ADDR, (unsigned char *)data_ptr,
				chunk_size, USB_TIMEOUT);

		if (ret != (int)chunk_size) {
			/* Abort if libusb sent less data than requested */
			return ERROR_FAIL;
		}

		bytes_remaining -= chunk_size;
		addr += chunk_size;
		data_ptr += chunk_size;
	}

	return ERROR_OK;
}

/************************** Generic helper functions **************************/

/**
 * Print state of interesting signals via LOG_INFO().
 *
 * @param input_signals input signal states as returned by CMD_GET_SIGNALS
 * @param output_signals output signal states as returned by CMD_GET_SIGNALS
 */
static void ulink_print_signal_states(uint8_t input_signals, uint8_t output_signals)
{
	LOG_INFO("ULINK signal states: TDI: %i, TDO: %i, TMS: %i, TCK: %i, TRST: %i,"
		" SRST: %i",
		(output_signals & SIGNAL_TDI   ? 1 : 0),
		(input_signals  & SIGNAL_TDO   ? 1 : 0),
		(output_signals & SIGNAL_TMS   ? 1 : 0),
		(output_signals & SIGNAL_TCK   ? 1 : 0),
		(output_signals & SIGNAL_TRST  ? 0 : 1),	/* Inverted by hardware */
		(output_signals & SIGNAL_RESET ? 0 : 1));	/* Inverted by hardware */
}

/**************** OpenULINK command generation helper functions ***************/

/**
 * Allocate and initialize space in memory for OpenULINK command payload.
 *
 * @param ulink_cmd pointer to command whose payload should be allocated.
 * @param size the amount of memory to allocate (bytes).
 * @param direction which payload to allocate.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_allocate_payload(struct ulink_cmd *ulink_cmd, int size,
	enum ulink_payload_direction direction)
{
	uint8_t *payload;

	payload = calloc(size, sizeof(uint8_t));

	if (payload == NULL) {
		LOG_ERROR("Could not allocate OpenULINK command payload: out of memory");
		return ERROR_FAIL;
	}

	switch (direction) {
	    case PAYLOAD_DIRECTION_OUT:
		    if (ulink_cmd->payload_out != NULL) {
			    LOG_ERROR("BUG: Duplicate payload allocation for OpenULINK command");
			    free(payload);
			    return ERROR_FAIL;
		    } else {
			    ulink_cmd->payload_out = payload;
			    ulink_cmd->payload_out_size = size;
		    }
		    break;
	    case PAYLOAD_DIRECTION_IN:
		    if (ulink_cmd->payload_in_start != NULL) {
			    LOG_ERROR("BUG: Duplicate payload allocation for OpenULINK command");
			    free(payload);
			    return ERROR_FAIL;
		    } else {
			    ulink_cmd->payload_in_start = payload;
			    ulink_cmd->payload_in = payload;
			    ulink_cmd->payload_in_size = size;

				/* By default, free payload_in_start in ulink_clear_queue(). Commands
				 * that do not want this behavior (e. g. split scans) must turn it off
				 * separately! */
			    ulink_cmd->free_payload_in_start = true;
		    }
		    break;
	}

	return ERROR_OK;
}

/****************** OpenULINK command queue helper functions ******************/

/**
 * Get the current number of bytes in the queue, including command IDs.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param direction the transfer direction for which to get byte count.
 * @return the number of bytes currently stored in the queue for the specified
 *  direction.
 */
static int ulink_get_queue_size(struct ulink *device,
	enum ulink_payload_direction direction)
{
	struct ulink_cmd *current = device->queue_start;
	int sum = 0;

	while (current != NULL) {
		switch (direction) {
		    case PAYLOAD_DIRECTION_OUT:
			    sum += current->payload_out_size + 1;	/* + 1 byte for Command ID */
			    break;
		    case PAYLOAD_DIRECTION_IN:
			    sum += current->payload_in_size;
			    break;
		}

		current = current->next;
	}

	return sum;
}

/**
 * Clear the OpenULINK command queue.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static void ulink_clear_queue(struct ulink *device)
{
	struct ulink_cmd *current = device->queue_start;
	struct ulink_cmd *next = NULL;

	while (current != NULL) {
		/* Save pointer to next element */
		next = current->next;

		/* Free payloads: OUT payload can be freed immediately */
		free(current->payload_out);
		current->payload_out = NULL;

		/* IN payload MUST be freed ONLY if no other commands use the
		 * payload_in_start buffer */
		if (current->free_payload_in_start == true) {
			free(current->payload_in_start);
			current->payload_in_start = NULL;
			current->payload_in = NULL;
		}

		/* Free queue element */
		free(current);

		/* Proceed with next element */
		current = next;
	}

	device->commands_in_queue = 0;
	device->queue_start = NULL;
	device->queue_end = NULL;
}

/**
 * Add a command to the OpenULINK command queue.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param ulink_cmd pointer to command that shall be appended to the OpenULINK
 *  command queue.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_queue(struct ulink *device, struct ulink_cmd *ulink_cmd)
{
	int newsize_out, newsize_in;
	int ret = ERROR_OK;

	newsize_out = ulink_get_queue_size(device, PAYLOAD_DIRECTION_OUT) + 1
		+ ulink_cmd->payload_out_size;

	newsize_in = ulink_get_queue_size(device, PAYLOAD_DIRECTION_IN)
		+ ulink_cmd->payload_in_size;

	/* Check if the current command can be appended to the queue */
	if ((newsize_out > 64) || (newsize_in > 64)) {
		/* New command does not fit. Execute all commands in queue before starting
		 * new queue with the current command as first entry. */
		ret = ulink_execute_queued_commands(device, USB_TIMEOUT);

		if (ret == ERROR_OK)
			ret = ulink_post_process_queue(device);

		if (ret == ERROR_OK)
			ulink_clear_queue(device);
	}

	if (device->queue_start == NULL) {
		/* Queue was empty */
		device->commands_in_queue = 1;

		device->queue_start = ulink_cmd;
		device->queue_end = ulink_cmd;
	} else {
		/* There are already commands in the queue */
		device->commands_in_queue++;

		device->queue_end->next = ulink_cmd;
		device->queue_end = ulink_cmd;
	}

	if (ret != ERROR_OK)
		ulink_clear_queue(device);

	return ret;
}

/**
 * Sends all queued OpenULINK commands to the ULINK for execution.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param timeout
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_execute_queued_commands(struct ulink *device, int timeout)
{
	struct ulink_cmd *current;
	int ret, i, index_out, index_in, count_out, count_in, transferred;
	uint8_t buffer[64];

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO))
		ulink_print_queue(device);

	index_out = 0;
	count_out = 0;
	count_in = 0;

	for (current = device->queue_start; current; current = current->next) {
		/* Add command to packet */
		buffer[index_out] = current->id;
		index_out++;
		count_out++;

		for (i = 0; i < current->payload_out_size; i++)
			buffer[index_out + i] = current->payload_out[i];
		index_out += current->payload_out_size;
		count_in += current->payload_in_size;
		count_out += current->payload_out_size;
	}

	/* Send packet to ULINK */
	ret = libusb_bulk_transfer(device->usb_device_handle, device->ep_out,
			(unsigned char *)buffer, count_out, &transferred, timeout);
	if (ret != 0)
		return ERROR_FAIL;
	if (transferred != count_out)
		return ERROR_FAIL;

	/* Wait for response if commands contain IN payload data */
	if (count_in > 0) {
		ret = libusb_bulk_transfer(device->usb_device_handle, device->ep_in,
				(unsigned char *)buffer, 64, &transferred, timeout);
		if (ret != 0)
			return ERROR_FAIL;
		if (transferred != count_in)
			return ERROR_FAIL;

		/* Write back IN payload data */
		index_in = 0;
		for (current = device->queue_start; current; current = current->next) {
			for (i = 0; i < current->payload_in_size; i++) {
				current->payload_in[i] = buffer[index_in];
				index_in++;
			}
		}
	}

	return ERROR_OK;
}

/**
 * Convert an OpenULINK command ID (\a id) to a human-readable string.
 *
 * @param id the OpenULINK command ID.
 * @return the corresponding human-readable string.
 */
static const char *ulink_cmd_id_string(uint8_t id)
{
	switch (id) {
	case CMD_SCAN_IN:
		return "CMD_SCAN_IN";
	case CMD_SLOW_SCAN_IN:
		return "CMD_SLOW_SCAN_IN";
	case CMD_SCAN_OUT:
		return "CMD_SCAN_OUT";
	case CMD_SLOW_SCAN_OUT:
		return "CMD_SLOW_SCAN_OUT";
	case CMD_SCAN_IO:
		return "CMD_SCAN_IO";
	case CMD_SLOW_SCAN_IO:
		return "CMD_SLOW_SCAN_IO";
	case CMD_CLOCK_TMS:
		return "CMD_CLOCK_TMS";
	case CMD_SLOW_CLOCK_TMS:
		return "CMD_SLOW_CLOCK_TMS";
	case CMD_CLOCK_TCK:
		return "CMD_CLOCK_TCK";
	case CMD_SLOW_CLOCK_TCK:
		return "CMD_SLOW_CLOCK_TCK";
	case CMD_SLEEP_US:
		return "CMD_SLEEP_US";
	case CMD_SLEEP_MS:
		return "CMD_SLEEP_MS";
	case CMD_GET_SIGNALS:
		return "CMD_GET_SIGNALS";
	case CMD_SET_SIGNALS:
		return "CMD_SET_SIGNALS";
	case CMD_CONFIGURE_TCK_FREQ:
		return "CMD_CONFIGURE_TCK_FREQ";
	case CMD_SET_LEDS:
		return "CMD_SET_LEDS";
	case CMD_TEST:
		return "CMD_TEST";
	default:
		return "CMD_UNKNOWN";
	}
}

/**
 * Print one OpenULINK command to stdout.
 *
 * @param ulink_cmd pointer to OpenULINK command.
 */
static void ulink_print_command(struct ulink_cmd *ulink_cmd)
{
	int i;

	printf("  %-22s | OUT size = %i, bytes = 0x",
		ulink_cmd_id_string(ulink_cmd->id), ulink_cmd->payload_out_size);

	for (i = 0; i < ulink_cmd->payload_out_size; i++)
		printf("%02X ", ulink_cmd->payload_out[i]);
	printf("\n                         | IN size  = %i\n",
		ulink_cmd->payload_in_size);
}

/**
 * Print the OpenULINK command queue to stdout.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 */
static void ulink_print_queue(struct ulink *device)
{
	struct ulink_cmd *current;

	printf("OpenULINK command queue:\n");

	for (current = device->queue_start; current; current = current->next)
		ulink_print_command(current);
}

/**
 * Perform JTAG scan
 *
 * Creates and appends a JTAG scan command to the OpenULINK command queue.
 * A JTAG scan consists of three steps:
 * - Move to the desired SHIFT state, depending on scan type (IR/DR scan).
 * - Shift TDI data into the JTAG chain, optionally reading the TDO pin.
 * - Move to the desired end state.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param scan_type the type of the scan (IN, OUT, IO (bidirectional)).
 * @param scan_size_bits number of bits to shift into the JTAG chain.
 * @param tdi pointer to array containing TDI data.
 * @param tdo_start pointer to first element of array where TDO data shall be
 *  stored. See #ulink_cmd for details.
 * @param tdo pointer to array where TDO data shall be stored
 * @param tms_count_start number of TMS state transitions to perform BEFORE
 *  shifting data into the JTAG chain.
 * @param tms_sequence_start sequence of TMS state transitions that will be
 *  performed BEFORE shifting data into the JTAG chain.
 * @param tms_count_end number of TMS state transitions to perform AFTER
 *  shifting data into the JTAG chain.
 * @param tms_sequence_end sequence of TMS state transitions that will be
 *  performed AFTER shifting data into the JTAG chain.
 * @param origin pointer to OpenOCD command that generated this scan command.
 * @param postprocess whether this command needs to be post-processed after
 *  execution.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_scan_cmd(struct ulink *device, enum scan_type scan_type,
	int scan_size_bits, uint8_t *tdi, uint8_t *tdo_start, uint8_t *tdo,
	uint8_t tms_count_start, uint8_t tms_sequence_start, uint8_t tms_count_end,
	uint8_t tms_sequence_end, struct jtag_command *origin, bool postprocess)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret, i, scan_size_bytes;
	uint8_t bits_last_byte;

	if (cmd == NULL)
		return ERROR_FAIL;

	/* Check size of command. USB buffer can hold 64 bytes, 1 byte is command ID,
	 * 5 bytes are setup data -> 58 remaining payload bytes for TDI data */
	if (scan_size_bits > (58 * 8)) {
		LOG_ERROR("BUG: Tried to create CMD_SCAN_IO OpenULINK command with too"
			" large payload");
		free(cmd);
		return ERROR_FAIL;
	}

	scan_size_bytes = DIV_ROUND_UP(scan_size_bits, 8);

	bits_last_byte = scan_size_bits % 8;
	if (bits_last_byte == 0)
		bits_last_byte = 8;

	/* Allocate out_payload depending on scan type */
	switch (scan_type) {
	    case SCAN_IN:
		    if (device->delay_scan_in < 0)
			    cmd->id = CMD_SCAN_IN;
		    else
			    cmd->id = CMD_SLOW_SCAN_IN;
		    ret = ulink_allocate_payload(cmd, 5, PAYLOAD_DIRECTION_OUT);
		    break;
	    case SCAN_OUT:
		    if (device->delay_scan_out < 0)
			    cmd->id = CMD_SCAN_OUT;
		    else
			    cmd->id = CMD_SLOW_SCAN_OUT;
		    ret = ulink_allocate_payload(cmd, scan_size_bytes + 5, PAYLOAD_DIRECTION_OUT);
		    break;
	    case SCAN_IO:
		    if (device->delay_scan_io < 0)
			    cmd->id = CMD_SCAN_IO;
		    else
			    cmd->id = CMD_SLOW_SCAN_IO;
		    ret = ulink_allocate_payload(cmd, scan_size_bytes + 5, PAYLOAD_DIRECTION_OUT);
		    break;
	    default:
		    LOG_ERROR("BUG: ulink_append_scan_cmd() encountered an unknown scan type");
		    ret = ERROR_FAIL;
		    break;
	}

	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	/* Build payload_out that is common to all scan types */
	cmd->payload_out[0] = scan_size_bytes & 0xFF;
	cmd->payload_out[1] = bits_last_byte & 0xFF;
	cmd->payload_out[2] = ((tms_count_start & 0x0F) << 4) | (tms_count_end & 0x0F);
	cmd->payload_out[3] = tms_sequence_start;
	cmd->payload_out[4] = tms_sequence_end;

	/* Setup payload_out for types with OUT transfer */
	if ((scan_type == SCAN_OUT) || (scan_type == SCAN_IO)) {
		for (i = 0; i < scan_size_bytes; i++)
			cmd->payload_out[i + 5] = tdi[i];
	}

	/* Setup payload_in pointers for types with IN transfer */
	if ((scan_type == SCAN_IN) || (scan_type == SCAN_IO)) {
		cmd->payload_in_start = tdo_start;
		cmd->payload_in = tdo;
		cmd->payload_in_size = scan_size_bytes;
	}

	cmd->needs_postprocessing = postprocess;
	cmd->cmd_origin = origin;

	/* For scan commands, we free payload_in_start only when the command is
	 * the last in a series of split commands or a stand-alone command */
	cmd->free_payload_in_start = postprocess;

	return ulink_append_queue(device, cmd);
}

/**
 * Perform TAP state transitions
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param count defines the number of TCK clock cycles generated (up to 8).
 * @param sequence defines the TMS pin levels for each state transition. The
 *  Least-Significant Bit is read first.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_clock_tms_cmd(struct ulink *device, uint8_t count,
	uint8_t sequence)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret;

	if (cmd == NULL)
		return ERROR_FAIL;

	if (device->delay_clock_tms < 0)
		cmd->id = CMD_CLOCK_TMS;
	else
		cmd->id = CMD_SLOW_CLOCK_TMS;

	/* CMD_CLOCK_TMS has two OUT payload bytes and zero IN payload bytes */
	ret = ulink_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = count;
	cmd->payload_out[1] = sequence;

	return ulink_append_queue(device, cmd);
}

/**
 * Generate a defined amount of TCK clock cycles
 *
 * All other JTAG signals are left unchanged.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param count the number of TCK clock cycles to generate.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_clock_tck_cmd(struct ulink *device, uint16_t count)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret;

	if (cmd == NULL)
		return ERROR_FAIL;

	if (device->delay_clock_tck < 0)
		cmd->id = CMD_CLOCK_TCK;
	else
		cmd->id = CMD_SLOW_CLOCK_TCK;

	/* CMD_CLOCK_TCK has two OUT payload bytes and zero IN payload bytes */
	ret = ulink_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = count & 0xff;
	cmd->payload_out[1] = (count >> 8) & 0xff;

	return ulink_append_queue(device, cmd);
}

/**
 * Read JTAG signals.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_get_signals_cmd(struct ulink *device)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret;

	if (cmd == NULL)
		return ERROR_FAIL;

	cmd->id = CMD_GET_SIGNALS;
	cmd->needs_postprocessing = true;

	/* CMD_GET_SIGNALS has two IN payload bytes */
	ret = ulink_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_IN);

	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	return ulink_append_queue(device, cmd);
}

/**
 * Arbitrarily set JTAG output signals.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param low defines which signals will be de-asserted. Each bit corresponds
 *  to a JTAG signal:
 *  - SIGNAL_TDI
 *  - SIGNAL_TMS
 *  - SIGNAL_TCK
 *  - SIGNAL_TRST
 *  - SIGNAL_BRKIN
 *  - SIGNAL_RESET
 *  - SIGNAL_OCDSE
 * @param high defines which signals will be asserted.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_set_signals_cmd(struct ulink *device, uint8_t low,
	uint8_t high)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret;

	if (cmd == NULL)
		return ERROR_FAIL;

	cmd->id = CMD_SET_SIGNALS;

	/* CMD_SET_SIGNALS has two OUT payload bytes and zero IN payload bytes */
	ret = ulink_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_OUT);

	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = low;
	cmd->payload_out[1] = high;

	return ulink_append_queue(device, cmd);
}

/**
 * Sleep for a pre-defined number of microseconds
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param us the number microseconds to sleep.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_sleep_cmd(struct ulink *device, uint32_t us)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret;

	if (cmd == NULL)
		return ERROR_FAIL;

	cmd->id = CMD_SLEEP_US;

	/* CMD_SLEEP_US has two OUT payload bytes and zero IN payload bytes */
	ret = ulink_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_OUT);

	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = us & 0x00ff;
	cmd->payload_out[1] = (us >> 8) & 0x00ff;

	return ulink_append_queue(device, cmd);
}

/**
 * Set TCK delay counters
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param delay_scan_in delay count top value in jtag_slow_scan_in() function.
 * @param delay_scan_out delay count top value in jtag_slow_scan_out() function.
 * @param delay_scan_io delay count top value in jtag_slow_scan_io() function.
 * @param delay_tck delay count top value in jtag_clock_tck() function.
 * @param delay_tms delay count top value in jtag_slow_clock_tms() function.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_configure_tck_cmd(struct ulink *device, int delay_scan_in,
	int delay_scan_out, int delay_scan_io, int delay_tck, int delay_tms)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret;

	if (cmd == NULL)
		return ERROR_FAIL;

	cmd->id = CMD_CONFIGURE_TCK_FREQ;

	/* CMD_CONFIGURE_TCK_FREQ has five OUT payload bytes and zero
	 * IN payload bytes */
	ret = ulink_allocate_payload(cmd, 5, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	if (delay_scan_in < 0)
		cmd->payload_out[0] = 0;
	else
		cmd->payload_out[0] = (uint8_t)delay_scan_in;

	if (delay_scan_out < 0)
		cmd->payload_out[1] = 0;
	else
		cmd->payload_out[1] = (uint8_t)delay_scan_out;

	if (delay_scan_io < 0)
		cmd->payload_out[2] = 0;
	else
		cmd->payload_out[2] = (uint8_t)delay_scan_io;

	if (delay_tck < 0)
		cmd->payload_out[3] = 0;
	else
		cmd->payload_out[3] = (uint8_t)delay_tck;

	if (delay_tms < 0)
		cmd->payload_out[4] = 0;
	else
		cmd->payload_out[4] = (uint8_t)delay_tms;

	return ulink_append_queue(device, cmd);
}

/**
 * Turn on/off ULINK LEDs.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param led_state which LED(s) to turn on or off. The following bits
 *  influence the LEDS:
 *  - Bit 0: Turn COM LED on
 *  - Bit 1: Turn RUN LED on
 *  - Bit 2: Turn COM LED off
 *  - Bit 3: Turn RUN LED off
 *  If both the on-bit and the off-bit for the same LED is set, the LED is
 *  turned off.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_led_cmd(struct ulink *device, uint8_t led_state)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret;

	if (cmd == NULL)
		return ERROR_FAIL;

	cmd->id = CMD_SET_LEDS;

	/* CMD_SET_LEDS has one OUT payload byte and zero IN payload bytes */
	ret = ulink_allocate_payload(cmd, 1, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = led_state;

	return ulink_append_queue(device, cmd);
}

/**
 * Test command. Used to check if the ULINK device is ready to accept new
 * commands.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_append_test_cmd(struct ulink *device)
{
	struct ulink_cmd *cmd = calloc(1, sizeof(struct ulink_cmd));
	int ret;

	if (cmd == NULL)
		return ERROR_FAIL;

	cmd->id = CMD_TEST;

	/* CMD_TEST has one OUT payload byte and zero IN payload bytes */
	ret = ulink_allocate_payload(cmd, 1, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = 0xAA;

	return ulink_append_queue(device, cmd);
}

/****************** OpenULINK TCK frequency helper functions ******************/

/**
 * Calculate delay values for a given TCK frequency.
 *
 * The OpenULINK firmware uses five different speed values for different
 * commands. These speed values are calculated in these functions.
 *
 * The five different commands which support variable TCK frequency are
 * implemented twice in the firmware:
 *   1. Maximum possible frequency without any artificial delay
 *   2. Variable frequency with artificial linear delay loop
 *
 * To set the ULINK to maximum frequency, it is only necessary to use the
 * corresponding command IDs. To set the ULINK to a lower frequency, the
 * delay loop top values have to be calculated first. Then, a
 * CMD_CONFIGURE_TCK_FREQ command needs to be sent to the ULINK device.
 *
 * The delay values are described by linear equations:
 *    t = k * x + d
 *    (t = period, k = constant, x = delay value, d = constant)
 *
 * Thus, the delay can be calculated as in the following equation:
 *    x = (t - d) / k
 *
 * The constants in these equations have been determined and validated by
 * measuring the frequency resulting from different delay values.
 *
 * @param type for which command to calculate the delay value.
 * @param f TCK frequency for which to calculate the delay value in Hz.
 * @param delay where to store resulting delay value.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_calculate_delay(enum ulink_delay_type type, long f, int *delay)
{
	float t, x, x_ceil;

	/* Calculate period of requested TCK frequency */
	t = 1.0 / (float)(f);

	switch (type) {
	    case DELAY_CLOCK_TCK:
		    x = (t - (float)(6E-6)) / (float)(4E-6);
		    break;
	    case DELAY_CLOCK_TMS:
		    x = (t - (float)(8.5E-6)) / (float)(4E-6);
		    break;
	    case DELAY_SCAN_IN:
		    x = (t - (float)(8.8308E-6)) / (float)(4E-6);
		    break;
	    case DELAY_SCAN_OUT:
		    x = (t - (float)(1.0527E-5)) / (float)(4E-6);
		    break;
	    case DELAY_SCAN_IO:
		    x = (t - (float)(1.3132E-5)) / (float)(4E-6);
		    break;
	    default:
		    return ERROR_FAIL;
		    break;
	}

	/* Check if the delay value is negative. This happens when a frequency is
	 * requested that is too high for the delay loop implementation. In this
	 * case, set delay value to zero. */
	if (x < 0)
		x = 0;

	/* We need to convert the exact delay value to an integer. Therefore, we
	 * round the exact value UP to ensure that the resulting frequency is NOT
	 * higher than the requested frequency. */
	x_ceil = ceilf(x);

	/* Check if the value is within limits */
	if (x_ceil > 255)
		return ERROR_FAIL;

	*delay = (int)x_ceil;

	return ERROR_OK;
}

/**
 * Calculate frequency for a given delay value.
 *
 * Similar to the #ulink_calculate_delay function, this function calculates the
 * TCK frequency for a given delay value by using linear equations of the form:
 *    t = k * x + d
 *    (t = period, k = constant, x = delay value, d = constant)
 *
 * @param type for which command to calculate the delay value.
 * @param delay delay value for which to calculate the resulting TCK frequency.
 * @return the resulting TCK frequency
 */
static long ulink_calculate_frequency(enum ulink_delay_type type, int delay)
{
	float t, f_float;

	if (delay > 255)
		return 0;

	switch (type) {
	    case DELAY_CLOCK_TCK:
		    if (delay < 0)
			    t = (float)(2.666E-6);
		    else
			    t = (float)(4E-6) * (float)(delay) + (float)(6E-6);
		    break;
	    case DELAY_CLOCK_TMS:
		    if (delay < 0)
			    t = (float)(5.666E-6);
		    else
			    t = (float)(4E-6) * (float)(delay) + (float)(8.5E-6);
		    break;
	    case DELAY_SCAN_IN:
		    if (delay < 0)
			    t = (float)(5.5E-6);
		    else
			    t = (float)(4E-6) * (float)(delay) + (float)(8.8308E-6);
		    break;
	    case DELAY_SCAN_OUT:
		    if (delay < 0)
			    t = (float)(7.0E-6);
		    else
			    t = (float)(4E-6) * (float)(delay) + (float)(1.0527E-5);
		    break;
	    case DELAY_SCAN_IO:
		    if (delay < 0)
			    t = (float)(9.926E-6);
		    else
			    t = (float)(4E-6) * (float)(delay) + (float)(1.3132E-5);
		    break;
	    default:
		    return 0;
	}

	f_float = 1.0 / t;
	return roundf(f_float);
}

/******************* Interface between OpenULINK and OpenOCD ******************/

/**
 * Sets the end state follower (see interface.h) if \a endstate is a stable
 * state.
 *
 * @param endstate the state the end state follower should be set to.
 */
static void ulink_set_end_state(tap_state_t endstate)
{
	if (tap_is_state_stable(endstate))
		tap_set_end_state(endstate);
	else {
		LOG_ERROR("BUG: %s is not a valid end state", tap_state_name(endstate));
		exit(EXIT_FAILURE);
	}
}

/**
 * Move from the current TAP state to the current TAP end state.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_queue_statemove(struct ulink *device)
{
	uint8_t tms_sequence, tms_count;
	int ret;

	if (tap_get_state() == tap_get_end_state()) {
		/* Do nothing if we are already there */
		return ERROR_OK;
	}

	tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	ret = ulink_append_clock_tms_cmd(device, tms_count, tms_sequence);

	if (ret == ERROR_OK)
		tap_set_state(tap_get_end_state());

	return ret;
}

/**
 * Perform a scan operation on a JTAG register.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_queue_scan(struct ulink *device, struct jtag_command *cmd)
{
	uint32_t scan_size_bits, scan_size_bytes, bits_last_scan;
	uint32_t scans_max_payload, bytecount;
	uint8_t *tdi_buffer_start = NULL, *tdi_buffer = NULL;
	uint8_t *tdo_buffer_start = NULL, *tdo_buffer = NULL;

	uint8_t first_tms_count, first_tms_sequence;
	uint8_t last_tms_count, last_tms_sequence;

	uint8_t tms_count_pause, tms_sequence_pause;
	uint8_t tms_count_resume, tms_sequence_resume;

	uint8_t tms_count_start, tms_sequence_start;
	uint8_t tms_count_end, tms_sequence_end;

	enum scan_type type;
	int ret;

	/* Determine scan size */
	scan_size_bits = jtag_scan_size(cmd->cmd.scan);
	scan_size_bytes = DIV_ROUND_UP(scan_size_bits, 8);

	/* Determine scan type (IN/OUT/IO) */
	type = jtag_scan_type(cmd->cmd.scan);

	/* Determine number of scan commands with maximum payload */
	scans_max_payload = scan_size_bytes / 58;

	/* Determine size of last shift command */
	bits_last_scan = scan_size_bits - (scans_max_payload * 58 * 8);

	/* Allocate TDO buffer if required */
	if ((type == SCAN_IN) || (type == SCAN_IO)) {
		tdo_buffer_start = calloc(sizeof(uint8_t), scan_size_bytes);

		if (tdo_buffer_start == NULL)
			return ERROR_FAIL;

		tdo_buffer = tdo_buffer_start;
	}

	/* Fill TDI buffer if required */
	if ((type == SCAN_OUT) || (type == SCAN_IO)) {
		jtag_build_buffer(cmd->cmd.scan, &tdi_buffer_start);
		tdi_buffer = tdi_buffer_start;
	}

	/* Get TAP state transitions */
	if (cmd->cmd.scan->ir_scan) {
		ulink_set_end_state(TAP_IRSHIFT);
		first_tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		first_tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());

		tap_set_state(TAP_IRSHIFT);
		tap_set_end_state(cmd->cmd.scan->end_state);
		last_tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		last_tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());

		/* TAP state transitions for split scans */
		tms_count_pause = tap_get_tms_path_len(TAP_IRSHIFT, TAP_IRPAUSE);
		tms_sequence_pause = tap_get_tms_path(TAP_IRSHIFT, TAP_IRPAUSE);
		tms_count_resume = tap_get_tms_path_len(TAP_IRPAUSE, TAP_IRSHIFT);
		tms_sequence_resume = tap_get_tms_path(TAP_IRPAUSE, TAP_IRSHIFT);
	} else {
		ulink_set_end_state(TAP_DRSHIFT);
		first_tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		first_tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());

		tap_set_state(TAP_DRSHIFT);
		tap_set_end_state(cmd->cmd.scan->end_state);
		last_tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		last_tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());

		/* TAP state transitions for split scans */
		tms_count_pause = tap_get_tms_path_len(TAP_DRSHIFT, TAP_DRPAUSE);
		tms_sequence_pause = tap_get_tms_path(TAP_DRSHIFT, TAP_DRPAUSE);
		tms_count_resume = tap_get_tms_path_len(TAP_DRPAUSE, TAP_DRSHIFT);
		tms_sequence_resume = tap_get_tms_path(TAP_DRPAUSE, TAP_DRSHIFT);
	}

	/* Generate scan commands */
	bytecount = scan_size_bytes;
	while (bytecount > 0) {
		if (bytecount == scan_size_bytes) {
			/* This is the first scan */
			tms_count_start = first_tms_count;
			tms_sequence_start = first_tms_sequence;
		} else {
			/* Resume from previous scan */
			tms_count_start = tms_count_resume;
			tms_sequence_start = tms_sequence_resume;
		}

		if (bytecount > 58) {	/* Full scan, at least one scan will follow */
			tms_count_end = tms_count_pause;
			tms_sequence_end = tms_sequence_pause;

			ret = ulink_append_scan_cmd(device,
					type,
					58 * 8,
					tdi_buffer,
					tdo_buffer_start,
					tdo_buffer,
					tms_count_start,
					tms_sequence_start,
					tms_count_end,
					tms_sequence_end,
					cmd,
					false);

			bytecount -= 58;

			/* Update TDI and TDO buffer pointers */
			if (tdi_buffer_start != NULL)
				tdi_buffer += 58;
			if (tdo_buffer_start != NULL)
				tdo_buffer += 58;
		} else if (bytecount == 58) {	/* Full scan, no further scans */
			tms_count_end = last_tms_count;
			tms_sequence_end = last_tms_sequence;

			ret = ulink_append_scan_cmd(device,
					type,
					58 * 8,
					tdi_buffer,
					tdo_buffer_start,
					tdo_buffer,
					tms_count_start,
					tms_sequence_start,
					tms_count_end,
					tms_sequence_end,
					cmd,
					true);

			bytecount = 0;
		} else {/* Scan with less than maximum payload, no further scans */
			tms_count_end = last_tms_count;
			tms_sequence_end = last_tms_sequence;

			ret = ulink_append_scan_cmd(device,
					type,
					bits_last_scan,
					tdi_buffer,
					tdo_buffer_start,
					tdo_buffer,
					tms_count_start,
					tms_sequence_start,
					tms_count_end,
					tms_sequence_end,
					cmd,
					true);

			bytecount = 0;
		}

		if (ret != ERROR_OK) {
			free(tdi_buffer_start);
			free(tdo_buffer_start);
			return ret;
		}
	}

	free(tdi_buffer_start);

	/* Set current state to the end state requested by the command */
	tap_set_state(cmd->cmd.scan->end_state);

	return ERROR_OK;
}

/**
 * Move the TAP into the Test Logic Reset state.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_queue_tlr_reset(struct ulink *device, struct jtag_command *cmd)
{
	int ret;

	ret = ulink_append_clock_tms_cmd(device, 5, 0xff);

	if (ret == ERROR_OK)
		tap_set_state(TAP_RESET);

	return ret;
}

/**
 * Run Test.
 *
 * Generate TCK clock cycles while remaining
 * in the Run-Test/Idle state.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_queue_runtest(struct ulink *device, struct jtag_command *cmd)
{
	int ret;

	/* Only perform statemove if the TAP currently isn't in the TAP_IDLE state */
	if (tap_get_state() != TAP_IDLE) {
		ulink_set_end_state(TAP_IDLE);
		ulink_queue_statemove(device);
	}

	/* Generate the clock cycles */
	ret = ulink_append_clock_tck_cmd(device, cmd->cmd.runtest->num_cycles);
	if (ret != ERROR_OK)
		return ret;

	/* Move to end state specified in command */
	if (cmd->cmd.runtest->end_state != tap_get_state()) {
		tap_set_end_state(cmd->cmd.runtest->end_state);
		ulink_queue_statemove(device);
	}

	return ERROR_OK;
}

/**
 * Execute a JTAG_RESET command
 *
 * @param device
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_queue_reset(struct ulink *device, struct jtag_command *cmd)
{
	uint8_t low = 0, high = 0;

	if (cmd->cmd.reset->trst) {
		tap_set_state(TAP_RESET);
		high |= SIGNAL_TRST;
	} else
		low |= SIGNAL_TRST;

	if (cmd->cmd.reset->srst)
		high |= SIGNAL_RESET;
	else
		low |= SIGNAL_RESET;

	return ulink_append_set_signals_cmd(device, low, high);
}

/**
 * Move to one TAP state or several states in succession.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_queue_pathmove(struct ulink *device, struct jtag_command *cmd)
{
	int ret, i, num_states, batch_size, state_count;
	tap_state_t *path;
	uint8_t tms_sequence;

	num_states = cmd->cmd.pathmove->num_states;
	path = cmd->cmd.pathmove->path;
	state_count = 0;

	while (num_states > 0) {
		tms_sequence = 0;

		/* Determine batch size */
		if (num_states >= 8)
			batch_size = 8;
		else
			batch_size = num_states;

		for (i = 0; i < batch_size; i++) {
			if (tap_state_transition(tap_get_state(), false) == path[state_count]) {
				/* Append '0' transition: clear bit 'i' in tms_sequence */
				buf_set_u32(&tms_sequence, i, 1, 0x0);
			} else if (tap_state_transition(tap_get_state(), true)
				   == path[state_count]) {
				/* Append '1' transition: set bit 'i' in tms_sequence */
				buf_set_u32(&tms_sequence, i, 1, 0x1);
			} else {
				/* Invalid state transition */
				LOG_ERROR("BUG: %s -> %s isn't a valid TAP state transition",
					tap_state_name(tap_get_state()),
					tap_state_name(path[state_count]));
				return ERROR_FAIL;
			}

			tap_set_state(path[state_count]);
			state_count++;
			num_states--;
		}

		/* Append CLOCK_TMS command to OpenULINK command queue */
		LOG_INFO(
			"pathmove batch: count = %i, sequence = 0x%x", batch_size, tms_sequence);
		ret = ulink_append_clock_tms_cmd(ulink_handle, batch_size, tms_sequence);
		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

/**
 * Sleep for a specific amount of time.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_queue_sleep(struct ulink *device, struct jtag_command *cmd)
{
	/* IMPORTANT! Due to the time offset in command execution introduced by
	 * command queueing, this needs to be implemented in the ULINK device */
	return ulink_append_sleep_cmd(device, cmd->cmd.sleep->us);
}

/**
 * Generate TCK cycles while remaining in a stable state.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @param cmd pointer to the command that shall be executed.
 */
static int ulink_queue_stableclocks(struct ulink *device, struct jtag_command *cmd)
{
	int ret;
	unsigned num_cycles;

	if (!tap_is_state_stable(tap_get_state())) {
		LOG_ERROR("JTAG_STABLECLOCKS: state not stable");
		return ERROR_FAIL;
	}

	num_cycles = cmd->cmd.stableclocks->num_cycles;

	/* TMS stays either high (Test Logic Reset state) or low (all other states) */
	if (tap_get_state() == TAP_RESET)
		ret = ulink_append_set_signals_cmd(device, 0, SIGNAL_TMS);
	else
		ret = ulink_append_set_signals_cmd(device, SIGNAL_TMS, 0);

	if (ret != ERROR_OK)
		return ret;

	while (num_cycles > 0) {
		if (num_cycles > 0xFFFF) {
			/* OpenULINK CMD_CLOCK_TCK can generate up to 0xFFFF (uint16_t) cycles */
			ret = ulink_append_clock_tck_cmd(device, 0xFFFF);
			num_cycles -= 0xFFFF;
		} else {
			ret = ulink_append_clock_tck_cmd(device, num_cycles);
			num_cycles = 0;
		}

		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

/**
 * Post-process JTAG_SCAN command
 *
 * @param ulink_cmd pointer to OpenULINK command that shall be processed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_post_process_scan(struct ulink_cmd *ulink_cmd)
{
	struct jtag_command *cmd = ulink_cmd->cmd_origin;
	int ret;

	switch (jtag_scan_type(cmd->cmd.scan)) {
	    case SCAN_IN:
	    case SCAN_IO:
		    ret = jtag_read_buffer(ulink_cmd->payload_in_start, cmd->cmd.scan);
		    break;
	    case SCAN_OUT:
			/* Nothing to do for OUT scans */
		    ret = ERROR_OK;
		    break;
	    default:
		    LOG_ERROR("BUG: ulink_post_process_scan() encountered an unknown"
			" JTAG scan type");
		    ret = ERROR_FAIL;
		    break;
	}

	return ret;
}

/**
 * Perform post-processing of commands after OpenULINK queue has been executed.
 *
 * @param device pointer to struct ulink identifying ULINK driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_post_process_queue(struct ulink *device)
{
	struct ulink_cmd *current;
	struct jtag_command *openocd_cmd;
	int ret;

	current = device->queue_start;

	while (current != NULL) {
		openocd_cmd = current->cmd_origin;

		/* Check if a corresponding OpenOCD command is stored for this
		 * OpenULINK command */
		if ((current->needs_postprocessing == true) && (openocd_cmd != NULL)) {
			switch (openocd_cmd->type) {
			    case JTAG_SCAN:
				    ret = ulink_post_process_scan(current);
				    break;
			    case JTAG_TLR_RESET:
			    case JTAG_RUNTEST:
			    case JTAG_RESET:
			    case JTAG_PATHMOVE:
			    case JTAG_SLEEP:
			    case JTAG_STABLECLOCKS:
					/* Nothing to do for these commands */
				    ret = ERROR_OK;
				    break;
			    default:
				    ret = ERROR_FAIL;
				    LOG_ERROR("BUG: ulink_post_process_queue() encountered unknown JTAG "
					"command type");
				    break;
			}

			if (ret != ERROR_OK)
				return ret;
		}

		current = current->next;
	}

	return ERROR_OK;
}

/**************************** JTAG driver functions ***************************/

/**
 * Executes the JTAG Command Queue.
 *
 * This is done in three stages: First, all OpenOCD commands are processed into
 * queued OpenULINK commands. Next, the OpenULINK command queue is sent to the
 * ULINK device and data received from the ULINK device is cached. Finally,
 * the post-processing function writes back data to the corresponding OpenOCD
 * commands.
 *
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int ret;

	while (cmd) {
		switch (cmd->type) {
		    case JTAG_SCAN:
			    ret = ulink_queue_scan(ulink_handle, cmd);
			    break;
		    case JTAG_TLR_RESET:
			    ret = ulink_queue_tlr_reset(ulink_handle, cmd);
			    break;
		    case JTAG_RUNTEST:
			    ret = ulink_queue_runtest(ulink_handle, cmd);
			    break;
		    case JTAG_RESET:
			    ret = ulink_queue_reset(ulink_handle, cmd);
			    break;
		    case JTAG_PATHMOVE:
			    ret = ulink_queue_pathmove(ulink_handle, cmd);
			    break;
		    case JTAG_SLEEP:
			    ret = ulink_queue_sleep(ulink_handle, cmd);
			    break;
		    case JTAG_STABLECLOCKS:
			    ret = ulink_queue_stableclocks(ulink_handle, cmd);
			    break;
		    default:
			    ret = ERROR_FAIL;
			    LOG_ERROR("BUG: encountered unknown JTAG command type");
			    break;
		}

		if (ret != ERROR_OK)
			return ret;

		cmd = cmd->next;
	}

	if (ulink_handle->commands_in_queue > 0) {
		ret = ulink_execute_queued_commands(ulink_handle, USB_TIMEOUT);
		if (ret != ERROR_OK)
			return ret;

		ret = ulink_post_process_queue(ulink_handle);
		if (ret != ERROR_OK)
			return ret;

		ulink_clear_queue(ulink_handle);
	}

	return ERROR_OK;
}

/**
 * Set the TCK frequency of the ULINK adapter.
 *
 * @param khz desired JTAG TCK frequency.
 * @param jtag_speed where to store corresponding adapter-specific speed value.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_khz(int khz, int *jtag_speed)
{
	int ret;

	if (khz == 0) {
		LOG_ERROR("RCLK not supported");
		return ERROR_FAIL;
	}

	/* CLOCK_TCK commands are decoupled from others. Therefore, the frequency
	 * setting can be done independently from all other commands. */
	if (khz >= 375)
		ulink_handle->delay_clock_tck = -1;
	else {
		ret = ulink_calculate_delay(DELAY_CLOCK_TCK, khz * 1000,
				&ulink_handle->delay_clock_tck);
		if (ret != ERROR_OK)
			return ret;
	}

	/* SCAN_{IN,OUT,IO} commands invoke CLOCK_TMS commands. Therefore, if the
	 * requested frequency goes below the maximum frequency for SLOW_CLOCK_TMS
	 * commands, all SCAN commands MUST also use the variable frequency
	 * implementation! */
	if (khz >= 176) {
		ulink_handle->delay_clock_tms = -1;
		ulink_handle->delay_scan_in = -1;
		ulink_handle->delay_scan_out = -1;
		ulink_handle->delay_scan_io = -1;
	} else {
		ret = ulink_calculate_delay(DELAY_CLOCK_TMS, khz * 1000,
				&ulink_handle->delay_clock_tms);
		if (ret != ERROR_OK)
			return ret;

		ret = ulink_calculate_delay(DELAY_SCAN_IN, khz * 1000,
				&ulink_handle->delay_scan_in);
		if (ret != ERROR_OK)
			return ret;

		ret = ulink_calculate_delay(DELAY_SCAN_OUT, khz * 1000,
				&ulink_handle->delay_scan_out);
		if (ret != ERROR_OK)
			return ret;

		ret = ulink_calculate_delay(DELAY_SCAN_IO, khz * 1000,
				&ulink_handle->delay_scan_io);
		if (ret != ERROR_OK)
			return ret;
	}

	LOG_DEBUG_IO("ULINK TCK setup: delay_tck      = %i (%li Hz),",
		ulink_handle->delay_clock_tck,
		ulink_calculate_frequency(DELAY_CLOCK_TCK, ulink_handle->delay_clock_tck));
	LOG_DEBUG_IO("                 delay_tms      = %i (%li Hz),",
		ulink_handle->delay_clock_tms,
		ulink_calculate_frequency(DELAY_CLOCK_TMS, ulink_handle->delay_clock_tms));
	LOG_DEBUG_IO("                 delay_scan_in  = %i (%li Hz),",
		ulink_handle->delay_scan_in,
		ulink_calculate_frequency(DELAY_SCAN_IN, ulink_handle->delay_scan_in));
	LOG_DEBUG_IO("                 delay_scan_out = %i (%li Hz),",
		ulink_handle->delay_scan_out,
		ulink_calculate_frequency(DELAY_SCAN_OUT, ulink_handle->delay_scan_out));
	LOG_DEBUG_IO("                 delay_scan_io  = %i (%li Hz),",
		ulink_handle->delay_scan_io,
		ulink_calculate_frequency(DELAY_SCAN_IO, ulink_handle->delay_scan_io));

	/* Configure the ULINK device with the new delay values */
	ret = ulink_append_configure_tck_cmd(ulink_handle,
			ulink_handle->delay_scan_in,
			ulink_handle->delay_scan_out,
			ulink_handle->delay_scan_io,
			ulink_handle->delay_clock_tck,
			ulink_handle->delay_clock_tms);

	if (ret != ERROR_OK)
		return ret;

	*jtag_speed = khz;

	return ERROR_OK;
}

/**
 * Set the TCK frequency of the ULINK adapter.
 *
 * Because of the way the TCK frequency is set up in the OpenULINK firmware,
 * there are five different speed settings. To simplify things, the
 * adapter-specific speed setting value is identical to the TCK frequency in
 * khz.
 *
 * @param speed desired adapter-specific speed value.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_speed(int speed)
{
	int dummy;

	return ulink_khz(speed, &dummy);
}

/**
 * Convert adapter-specific speed value to corresponding TCK frequency in kHz.
 *
 * Because of the way the TCK frequency is set up in the OpenULINK firmware,
 * there are five different speed settings. To simplify things, the
 * adapter-specific speed setting value is identical to the TCK frequency in
 * khz.
 *
 * @param speed adapter-specific speed value.
 * @param khz where to store corresponding TCK frequency in kHz.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_speed_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}

/**
 * Initiates the firmware download to the ULINK adapter and prepares
 * the USB handle.
 *
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_init(void)
{
	int ret, transferred;
	char str_manufacturer[20];
	bool download_firmware = false;
	unsigned char *dummy;
	uint8_t input_signals, output_signals;

	ulink_handle = calloc(1, sizeof(struct ulink));
	if (ulink_handle == NULL)
		return ERROR_FAIL;

	libusb_init(&ulink_handle->libusb_ctx);

	ret = ulink_usb_open(&ulink_handle);
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not open ULINK device");
		free(ulink_handle);
		ulink_handle = NULL;
		return ret;
	}

	/* Get String Descriptor to determine if firmware needs to be loaded */
	ret = libusb_get_string_descriptor_ascii(ulink_handle->usb_device_handle, 1, (unsigned char *)str_manufacturer, 20);
	if (ret < 0) {
		/* Could not get descriptor -> Unconfigured or original Keil firmware */
		download_firmware = true;
	} else {
		/* We got a String Descriptor, check if it is the correct one */
		if (strncmp(str_manufacturer, "OpenULINK", 9) != 0)
			download_firmware = true;
	}

	if (download_firmware == true) {
		LOG_INFO("Loading OpenULINK firmware. This is reversible by power-cycling"
			" ULINK device.");
		ret = ulink_load_firmware_and_renumerate(&ulink_handle,
				ULINK_FIRMWARE_FILE, ULINK_RENUMERATION_DELAY);
		if (ret != ERROR_OK) {
			LOG_ERROR("Could not download firmware and re-numerate ULINK");
			free(ulink_handle);
			ulink_handle = NULL;
			return ret;
		}
	} else
		LOG_INFO("ULINK device is already running OpenULINK firmware");

	/* Get OpenULINK USB IN/OUT endpoints and claim the interface */
	ret = jtag_libusb_choose_interface(ulink_handle->usb_device_handle,
		&ulink_handle->ep_in, &ulink_handle->ep_out, -1, -1, -1, -1);
	if (ret != ERROR_OK)
		return ret;

	/* Initialize OpenULINK command queue */
	ulink_clear_queue(ulink_handle);

	/* Issue one test command with short timeout */
	ret = ulink_append_test_cmd(ulink_handle);
	if (ret != ERROR_OK)
		return ret;

	ret = ulink_execute_queued_commands(ulink_handle, 200);
	if (ret != ERROR_OK) {
		/* Sending test command failed. The ULINK device may be forever waiting for
		 * the host to fetch an USB Bulk IN packet (e. g. OpenOCD crashed or was
		 * shut down by the user via Ctrl-C. Try to retrieve this Bulk IN packet. */
		dummy = calloc(64, sizeof(uint8_t));

		ret = libusb_bulk_transfer(ulink_handle->usb_device_handle, ulink_handle->ep_in,
				dummy, 64, &transferred, 200);

		free(dummy);

		if (ret != 0 || transferred == 0) {
			/* Bulk IN transfer failed -> unrecoverable error condition */
			LOG_ERROR("Cannot communicate with ULINK device. Disconnect ULINK from "
				"the USB port and re-connect, then re-run OpenOCD");
			free(ulink_handle);
			ulink_handle = NULL;
			return ERROR_FAIL;
		}
#ifdef _DEBUG_USB_COMMS_
		else {
			/* Successfully received Bulk IN packet -> continue */
			LOG_INFO("Recovered from lost Bulk IN packet");
		}
#endif
	}
	ulink_clear_queue(ulink_handle);

	ret = ulink_append_get_signals_cmd(ulink_handle);
	if (ret == ERROR_OK)
		ret = ulink_execute_queued_commands(ulink_handle, 200);

	if (ret == ERROR_OK) {
		/* Post-process the single CMD_GET_SIGNALS command */
		input_signals = ulink_handle->queue_start->payload_in[0];
		output_signals = ulink_handle->queue_start->payload_in[1];

		ulink_print_signal_states(input_signals, output_signals);
	}

	ulink_clear_queue(ulink_handle);

	return ERROR_OK;
}

/**
 * Closes the USB handle for the ULINK device.
 *
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int ulink_quit(void)
{
	int ret;

	ret = ulink_usb_close(&ulink_handle);
	free(ulink_handle);

	return ret;
}

/**
 * Set a custom path to ULINK firmware image and force downloading to ULINK.
 */
COMMAND_HANDLER(ulink_download_firmware_handler)
{
	int ret;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;


	LOG_INFO("Downloading ULINK firmware image %s", CMD_ARGV[0]);

	/* Download firmware image in CMD_ARGV[0] */
	ret = ulink_load_firmware_and_renumerate(&ulink_handle, CMD_ARGV[0],
			ULINK_RENUMERATION_DELAY);

	return ret;
}

/*************************** Command Registration **************************/

static const struct command_registration ulink_command_handlers[] = {
	{
		.name = "ulink_download_firmware",
		.handler = &ulink_download_firmware_handler,
		.mode = COMMAND_EXEC,
		.help = "download firmware image to ULINK device",
		.usage = "path/to/ulink_firmware.hex",
	},
	COMMAND_REGISTRATION_DONE,
};

static struct jtag_interface ulink_interface = {
	.execute_queue = ulink_execute_queue,
};

struct adapter_driver ulink_adapter_driver = {
	.name = "ulink",
	.transports = jtag_only,
	.commands = ulink_command_handlers,

	.init = ulink_init,
	.quit = ulink_quit,
	.speed = ulink_speed,
	.khz = ulink_khz,
	.speed_div = ulink_speed_div,

	.jtag_ops = &ulink_interface,
};
