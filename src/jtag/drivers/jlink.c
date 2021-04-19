/***************************************************************************
 *   Copyright (C) 2007 by Juergen Stuber <juergen@jstuber.net>            *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Jean-Christophe PLAGNIOL-VIILARD                *
 *   plagnioj@jcrosoft.com                                                 *
 *                                                                         *
 *   Copyright (C) 2015 by Marc Schink                                     *
 *   openocd-dev@marcschink.de                                             *
 *                                                                         *
 *   Copyright (C) 2015 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
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

#include <stdint.h>
#include <math.h>

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>
#include <jtag/drivers/jtag_usb_common.h>
#include <src/helper/replacements.h>
#include <target/cortex_m.h>

#include <libjaylink/libjaylink.h>

static struct jaylink_context *jayctx;
static struct jaylink_device_handle *devh;
static struct jaylink_connection conn;
static struct jaylink_connection connlist[JAYLINK_MAX_CONNECTIONS];
static enum jaylink_jtag_version jtag_command_version;
static uint8_t caps[JAYLINK_DEV_EXT_CAPS_SIZE];

static uint32_t serial_number;
static bool use_serial_number;
static bool use_usb_location;
static enum jaylink_usb_address usb_address;
static bool use_usb_address;
static enum jaylink_target_interface iface = JAYLINK_TIF_JTAG;
static bool trace_enabled;

#define JLINK_MAX_SPEED			12000
#define JLINK_TAP_BUFFER_SIZE	2048

static unsigned int swd_buffer_size = JLINK_TAP_BUFFER_SIZE;

/* Maximum SWO frequency deviation. */
#define SWO_MAX_FREQ_DEV	0.03

/* 256 byte non-volatile memory */
struct device_config {
	uint8_t	usb_address;
	/* 0ffset 0x01 to 0x03 */
	uint8_t	reserved_1[3];
	uint32_t target_power;
	/* 0ffset 0x08 to 0x1f */
	uint8_t reserved_2[24];
	/* IP only for J-Link Pro */
	uint8_t ip_address[4];
	uint8_t subnet_mask[4];
	/* 0ffset 0x28 to 0x2f */
	uint8_t reserved_3[8];
	uint8_t mac_address[6];
	/* 0ffset 0x36 to 0xff */
	uint8_t reserved_4[202];
} __attribute__ ((packed));

static struct device_config config;
static struct device_config tmp_config;

/* Queue command functions */
static void jlink_end_state(tap_state_t state);
static void jlink_state_move(void);
static void jlink_path_move(int num_states, tap_state_t *path);
static void jlink_stableclocks(int num_cycles);
static void jlink_runtest(int num_cycles);
static void jlink_reset(int trst, int srst);
static int jlink_reset_safe(int trst, int srst);
static int jlink_swd_run_queue(void);
static void jlink_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk);
static int jlink_swd_switch_seq(enum swd_special_seq seq);

/* J-Link tap buffer functions */
static void jlink_tap_init(void);
static int jlink_flush(void);
/**
 * Queue data to go out and in, flushing the queue as many times as
 * necessary.
 *
 * @param out A pointer to TDI data, if NULL, old stale data will be used.
 * @param out_offset A bit offset for TDI data.
 * @param tms_out A pointer to TMS data, if NULL, zeroes will be emitted.
 * @param tms_offset A bit offset for TMS data.
 * @param in A pointer to store TDO data to, if NULL the data will be discarded.
 * @param in_offset A bit offset for TDO data.
 * @param length Amount of bits to transfer out and in.
 *
 * @retval This function doesn't return any value.
 */
static void jlink_clock_data(const uint8_t *out, unsigned out_offset,
			     const uint8_t *tms_out, unsigned tms_offset,
			     uint8_t *in, unsigned in_offset,
			     unsigned length);

static enum tap_state jlink_last_state = TAP_RESET;
static int queued_retval;

/***************************************************************************/
/* External interface implementation */

static void jlink_execute_stableclocks(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("stableclocks %i cycles", cmd->cmd.runtest->num_cycles);
	jlink_stableclocks(cmd->cmd.runtest->num_cycles);
}

static void jlink_execute_runtest(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles,
		cmd->cmd.runtest->end_state);

	jlink_end_state(cmd->cmd.runtest->end_state);
	jlink_runtest(cmd->cmd.runtest->num_cycles);
}

static void jlink_execute_statemove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

	jlink_end_state(cmd->cmd.statemove->end_state);
	jlink_state_move();
}

static void jlink_execute_pathmove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("pathmove: %i states, end in %i",
		cmd->cmd.pathmove->num_states,
		cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	jlink_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
}

static void jlink_execute_scan(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("%s type:%d", cmd->cmd.scan->ir_scan ? "IRSCAN" : "DRSCAN",
		jtag_scan_type(cmd->cmd.scan));

	/* Make sure there are no trailing fields with num_bits == 0, or the logic below will fail. */
	while (cmd->cmd.scan->num_fields > 0
			&& cmd->cmd.scan->fields[cmd->cmd.scan->num_fields - 1].num_bits == 0) {
		cmd->cmd.scan->num_fields--;
		LOG_DEBUG("discarding trailing empty field");
	}

	if (cmd->cmd.scan->num_fields == 0) {
		LOG_DEBUG("empty scan, doing nothing");
		return;
	}

	if (cmd->cmd.scan->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT) {
			jlink_end_state(TAP_IRSHIFT);
			jlink_state_move();
		}
	} else {
		if (tap_get_state() != TAP_DRSHIFT) {
			jlink_end_state(TAP_DRSHIFT);
			jlink_state_move();
		}
	}

	jlink_end_state(cmd->cmd.scan->end_state);

	struct scan_field *field = cmd->cmd.scan->fields;
	unsigned scan_size = 0;

	for (int i = 0; i < cmd->cmd.scan->num_fields; i++, field++) {
		scan_size += field->num_bits;
		LOG_DEBUG_IO("%s%s field %d/%d %d bits",
			field->in_value ? "in" : "",
			field->out_value ? "out" : "",
			i,
			cmd->cmd.scan->num_fields,
			field->num_bits);

		if (i == cmd->cmd.scan->num_fields - 1 && tap_get_state() != tap_get_end_state()) {
			/* Last field, and we're leaving IRSHIFT/DRSHIFT. Clock last bit during tap
			 * movement. This last field can't have length zero, it was checked above. */
			jlink_clock_data(field->out_value,
					 0,
					 NULL,
					 0,
					 field->in_value,
					 0,
					 field->num_bits - 1);
			uint8_t last_bit = 0;
			if (field->out_value)
				bit_copy(&last_bit, 0, field->out_value, field->num_bits - 1, 1);
			uint8_t tms_bits = 0x01;
			jlink_clock_data(&last_bit,
					 0,
					 &tms_bits,
					 0,
					 field->in_value,
					 field->num_bits - 1,
					 1);
			tap_set_state(tap_state_transition(tap_get_state(), 1));
			jlink_clock_data(NULL,
					 0,
					 &tms_bits,
					 1,
					 NULL,
					 0,
					 1);
			tap_set_state(tap_state_transition(tap_get_state(), 0));
		} else
			jlink_clock_data(field->out_value,
					 0,
					 NULL,
					 0,
					 field->in_value,
					 0,
					 field->num_bits);
	}

	if (tap_get_state() != tap_get_end_state()) {
		jlink_end_state(tap_get_end_state());
		jlink_state_move();
	}

	LOG_DEBUG_IO("%s scan, %i bits, end in %s",
		(cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
		tap_state_name(tap_get_end_state()));
}

static void jlink_execute_sleep(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("sleep %" PRIu32 "", cmd->cmd.sleep->us);
	jlink_flush();
	jtag_sleep(cmd->cmd.sleep->us);
}

static int jlink_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_STABLECLOCKS:
			jlink_execute_stableclocks(cmd);
			break;
		case JTAG_RUNTEST:
			jlink_execute_runtest(cmd);
			break;
		case JTAG_TLR_RESET:
			jlink_execute_statemove(cmd);
			break;
		case JTAG_PATHMOVE:
			jlink_execute_pathmove(cmd);
			break;
		case JTAG_SCAN:
			jlink_execute_scan(cmd);
			break;
		case JTAG_SLEEP:
			jlink_execute_sleep(cmd);
			break;
		default:
			LOG_ERROR("BUG: Unknown JTAG command type encountered.");
			return ERROR_JTAG_QUEUE_FAILED;
	}

	return ERROR_OK;
}

static int jlink_execute_queue(void)
{
	int ret;
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		ret = jlink_execute_command(cmd);

		if (ret != ERROR_OK)
			return ret;

		cmd = cmd->next;
	}

	return jlink_flush();
}

static int jlink_speed(int speed)
{
	int ret;
	struct jaylink_speed tmp;
	int max_speed;

	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_GET_SPEEDS)) {
		ret = jaylink_get_speeds(devh, &tmp);

		if (ret != JAYLINK_OK) {
			LOG_ERROR("jaylink_get_speeds() failed: %s.",
				jaylink_strerror(ret));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		tmp.freq /= 1000;
		max_speed = tmp.freq / tmp.div;
	} else {
		max_speed = JLINK_MAX_SPEED;
	}

	if (!speed) {
		if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_ADAPTIVE_CLOCKING)) {
			LOG_ERROR("Adaptive clocking is not supported by the device.");
			return ERROR_JTAG_NOT_IMPLEMENTED;
		}

		speed = JAYLINK_SPEED_ADAPTIVE_CLOCKING;
	} else if (speed > max_speed) {
		LOG_INFO("Reduced speed from %d kHz to %d kHz (maximum).", speed,
			max_speed);
		speed = max_speed;
	}

	ret = jaylink_set_speed(devh, speed);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_set_speed() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int jlink_speed_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}

static int jlink_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;

	return ERROR_OK;
}

static bool read_device_config(struct device_config *cfg)
{
	int ret;

	ret = jaylink_read_raw_config(devh, (uint8_t *)cfg);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_read_raw_config() failed: %s.",
			jaylink_strerror(ret));
		return false;
	}

	if (cfg->usb_address == 0xff)
		cfg->usb_address = 0x00;

	if (cfg->target_power == 0xffffffff)
		cfg->target_power = 0;

	return true;
}

static int select_interface(void)
{
	int ret;
	uint32_t interfaces;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_SELECT_TIF)) {
		if (iface != JAYLINK_TIF_JTAG) {
			LOG_ERROR("Device supports JTAG transport only.");
			return ERROR_JTAG_INIT_FAILED;
		}

		return ERROR_OK;
	}

	ret = jaylink_get_available_interfaces(devh, &interfaces);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_get_available_interfaces() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_JTAG_INIT_FAILED;
	}

	if (!(interfaces & (1 << iface))) {
		LOG_ERROR("Selected transport is not supported by the device.");
		return ERROR_JTAG_INIT_FAILED;
	}

	ret = jaylink_select_interface(devh, iface, NULL);

	if (ret < 0) {
		LOG_ERROR("jaylink_select_interface() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int jlink_register(void)
{
	int ret;
	size_t i;
	bool handle_found;
	size_t count;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_REGISTER))
		return ERROR_OK;

	ret = jaylink_register(devh, &conn, connlist, &count);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_register() failed: %s.", jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	handle_found = false;

	for (i = 0; i < count; i++) {
		if (connlist[i].handle == conn.handle) {
			handle_found = true;
			break;
		}
	}

	if (!handle_found) {
		LOG_ERROR("Registration failed: maximum number of connections on the "
			"device reached.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/*
 * Adjust the SWD transaction buffer size depending on the free device internal
 * memory. This ensures that the SWD transactions sent to the device do not
 * exceed the internal memory of the device.
 */
static bool adjust_swd_buffer_size(void)
{
	int ret;
	uint32_t tmp;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_GET_FREE_MEMORY))
		return true;

	ret = jaylink_get_free_memory(devh, &tmp);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_get_free_memory() failed: %s.",
			jaylink_strerror(ret));
		return false;
	}

	if (tmp < 143) {
		LOG_ERROR("Not enough free device internal memory: %" PRIu32 " bytes.", tmp);
		return false;
	}

	tmp = MIN(JLINK_TAP_BUFFER_SIZE, (tmp - 16) / 2);

	if (tmp != swd_buffer_size) {
		swd_buffer_size = tmp;
		LOG_DEBUG("Adjusted SWD transaction buffer size to %u bytes.",
			swd_buffer_size);
	}

	return true;
}

static int jaylink_log_handler(const struct jaylink_context *ctx,
		enum jaylink_log_level level, const char *format, va_list args,
		void *user_data)
{
	enum log_levels tmp;

	switch (level) {
	case JAYLINK_LOG_LEVEL_ERROR:
		tmp = LOG_LVL_ERROR;
		break;
	case JAYLINK_LOG_LEVEL_WARNING:
		tmp = LOG_LVL_WARNING;
		break;
	/*
	 * Forward info messages to the debug output because they are more verbose
	 * than info messages of OpenOCD.
	 */
	case JAYLINK_LOG_LEVEL_INFO:
	case JAYLINK_LOG_LEVEL_DEBUG:
		tmp = LOG_LVL_DEBUG;
		break;
	case JAYLINK_LOG_LEVEL_DEBUG_IO:
		tmp = LOG_LVL_DEBUG_IO;
		break;
	default:
		tmp = LOG_LVL_WARNING;
	}

	log_vprintf_lf(tmp, __FILE__, __LINE__, __func__, format, args);

	return 0;
}

static bool jlink_usb_location_equal(struct jaylink_device *dev)
{
	int retval;
	uint8_t bus;
	uint8_t *ports;
	size_t num_ports;
	bool equal = false;

	retval = jaylink_device_get_usb_bus_ports(dev, &bus, &ports, &num_ports);

	if (retval == JAYLINK_ERR_NOT_SUPPORTED) {
		return false;
	} else if (retval != JAYLINK_OK) {
		LOG_WARNING("jaylink_device_get_usb_bus_ports() failed: %s.",
			jaylink_strerror(retval));
		return false;
	}

	equal = jtag_usb_location_equal(bus, ports,	num_ports);
	free(ports);

	return equal;
}


static int jlink_open_device(uint32_t ifaces, bool *found_device)
{
	int ret = jaylink_discovery_scan(jayctx, ifaces);
	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_discovery_scan() failed: %s.", jaylink_strerror(ret));
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	}

	size_t num_devices;
	struct jaylink_device **devs;
	ret = jaylink_get_devices(jayctx, &devs, &num_devices);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_get_devices() failed: %s.", jaylink_strerror(ret));
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	}

	use_usb_location = (jtag_usb_get_location() != NULL);

	if (!use_serial_number && !use_usb_address && !use_usb_location && num_devices > 1) {
		LOG_ERROR("Multiple devices found, specify the desired device.");
		jaylink_free_devices(devs, true);
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	}

	*found_device = false;

	for (size_t i = 0; devs[i]; i++) {
		struct jaylink_device *dev = devs[i];

		if (use_serial_number) {
			uint32_t tmp;
			ret = jaylink_device_get_serial_number(dev, &tmp);

			if (ret == JAYLINK_ERR_NOT_AVAILABLE) {
				continue;
			} else if (ret != JAYLINK_OK) {
				LOG_WARNING("jaylink_device_get_serial_number() failed: %s.",
					jaylink_strerror(ret));
				continue;
			}

			if (serial_number != tmp)
				continue;
		}

		if (use_usb_address) {
			enum jaylink_usb_address address;
			ret = jaylink_device_get_usb_address(dev, &address);

			if (ret == JAYLINK_ERR_NOT_SUPPORTED) {
				continue;
			} else if (ret != JAYLINK_OK) {
				LOG_WARNING("jaylink_device_get_usb_address() failed: %s.",
					jaylink_strerror(ret));
				continue;
			}

			if (usb_address != address)
				continue;
		}

		if (use_usb_location && !jlink_usb_location_equal(dev))
			continue;

		ret = jaylink_open(dev, &devh);

		if (ret == JAYLINK_OK) {
			*found_device = true;
			break;
		}

		LOG_ERROR("Failed to open device: %s.", jaylink_strerror(ret));
	}

	jaylink_free_devices(devs, true);
	return ERROR_OK;
}


static int jlink_init(void)
{
	int ret;
	char *firmware_version;
	struct jaylink_hardware_version hwver;
	struct jaylink_hardware_status hwstatus;
	size_t length;

	LOG_DEBUG("Using libjaylink %s (compiled with %s).",
		jaylink_version_package_get_string(), JAYLINK_VERSION_PACKAGE_STRING);

	if (!jaylink_library_has_cap(JAYLINK_CAP_HIF_USB) && use_usb_address) {
		LOG_ERROR("J-Link driver does not support USB devices.");
		return ERROR_JTAG_INIT_FAILED;
	}

	ret = jaylink_init(&jayctx);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_init() failed: %s.", jaylink_strerror(ret));
		return ERROR_JTAG_INIT_FAILED;
	}

	ret = jaylink_log_set_callback(jayctx, &jaylink_log_handler, NULL);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_log_set_callback() failed: %s.",
			jaylink_strerror(ret));
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	}

	bool found_device;
	ret = jlink_open_device(JAYLINK_HIF_USB, &found_device);
	if (ret != ERROR_OK)
		return ret;

	if (!found_device && use_serial_number) {
		ret = jlink_open_device(JAYLINK_HIF_TCP, &found_device);
		if (ret != ERROR_OK)
			return ret;
	}

	if (!found_device) {
		LOG_ERROR("No J-Link device found.");
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	}

	/*
	 * Be careful with changing the following initialization sequence because
	 * some devices are known to be sensitive regarding the order.
	 */

	ret = jaylink_get_firmware_version(devh, &firmware_version, &length);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_get_firmware_version() failed: %s.",
			jaylink_strerror(ret));
		jaylink_close(devh);
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	} else if (length > 0) {
		LOG_INFO("%s", firmware_version);
		free(firmware_version);
	} else {
		LOG_WARNING("Device responds empty firmware version string.");
	}

	memset(caps, 0, JAYLINK_DEV_EXT_CAPS_SIZE);
	ret = jaylink_get_caps(devh, caps);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_get_caps() failed: %s.", jaylink_strerror(ret));
		jaylink_close(devh);
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_GET_EXT_CAPS)) {
		ret = jaylink_get_extended_caps(devh, caps);

		if (ret != JAYLINK_OK) {
			LOG_ERROR("jaylink_get_extended_caps() failed:  %s.",
				jaylink_strerror(ret));
			jaylink_close(devh);
			jaylink_exit(jayctx);
			return ERROR_JTAG_INIT_FAILED;
		}
	}

	jtag_command_version = JAYLINK_JTAG_VERSION_2;

	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_GET_HW_VERSION)) {
		ret = jaylink_get_hardware_version(devh, &hwver);

		if (ret != JAYLINK_OK) {
			LOG_ERROR("Failed to retrieve hardware version: %s.",
				jaylink_strerror(ret));
			jaylink_close(devh);
			jaylink_exit(jayctx);
			return ERROR_JTAG_INIT_FAILED;
		}

		LOG_INFO("Hardware version: %u.%02u", hwver.major, hwver.minor);

		if (hwver.major >= 5)
			jtag_command_version = JAYLINK_JTAG_VERSION_3;
	}

	if (iface == JAYLINK_TIF_SWD) {
		/*
		 * Adjust the SWD transaction buffer size in case there is already
		 * allocated memory on the device. This happens for example if the
		 * memory for SWO capturing is still allocated because the software
		 * which used the device before has not been shut down properly.
		 */
		if (!adjust_swd_buffer_size()) {
			jaylink_close(devh);
			jaylink_exit(jayctx);
			return ERROR_JTAG_INIT_FAILED;
		}
	}

	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_READ_CONFIG)) {
		if (!read_device_config(&config)) {
			LOG_ERROR("Failed to read device configuration data.");
			jaylink_close(devh);
			jaylink_exit(jayctx);
			return ERROR_JTAG_INIT_FAILED;
		}

		memcpy(&tmp_config, &config, sizeof(struct device_config));
	}

	ret = jaylink_get_hardware_status(devh, &hwstatus);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_get_hardware_status() failed: %s.",
			jaylink_strerror(ret));
		jaylink_close(devh);
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_INFO("VTarget = %u.%03u V", hwstatus.target_voltage / 1000,
		hwstatus.target_voltage % 1000);

	conn.handle = 0;
	conn.pid = 0;
	strcpy(conn.hid, "0.0.0.0");
	conn.iid = 0;
	conn.cid = 0;

	ret = jlink_register();

	if (ret != ERROR_OK) {
		jaylink_close(devh);
		jaylink_exit(jayctx);
		return ERROR_JTAG_INIT_FAILED;
	}

	ret = select_interface();

	if (ret != ERROR_OK) {
		jaylink_close(devh);
		jaylink_exit(jayctx);
		return ret;
	}

	jlink_reset(0, 0);
	jtag_sleep(3000);
	jlink_tap_init();

	jlink_speed(jtag_get_speed_khz());

	if (iface == JAYLINK_TIF_JTAG) {
		/*
		 * J-Link devices with firmware version v5 and v6 seems to have an issue
		 * if the first tap move is not divisible by 8, so we send a TLR on
		 * first power up.
		 */
		uint8_t tms = 0xff;
		jlink_clock_data(NULL, 0, &tms, 0, NULL, 0, 8);

		jlink_flush();
	}

	return ERROR_OK;
}

static int jlink_quit(void)
{
	int ret;
	size_t count;

	if (trace_enabled) {
		ret = jaylink_swo_stop(devh);

		if (ret != JAYLINK_OK)
			LOG_ERROR("jaylink_swo_stop() failed: %s.", jaylink_strerror(ret));
	}

	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_REGISTER)) {
		ret = jaylink_unregister(devh, &conn, connlist, &count);

		if (ret != JAYLINK_OK)
			LOG_ERROR("jaylink_unregister() failed: %s.",
				jaylink_strerror(ret));
	}

	jaylink_close(devh);
	jaylink_exit(jayctx);

	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

static void jlink_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

/* Goes to the end state. */
static void jlink_state_move(void)
{
	uint8_t tms_scan;
	uint8_t tms_scan_bits;

	tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	jlink_clock_data(NULL, 0, &tms_scan, 0, NULL, 0, tms_scan_bits);

	tap_set_state(tap_get_end_state());
}

static void jlink_path_move(int num_states, tap_state_t *path)
{
	int i;
	uint8_t tms = 0xff;

	for (i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false))
			jlink_clock_data(NULL, 0, NULL, 0, NULL, 0, 1);
		else if (path[i] == tap_state_transition(tap_get_state(), true))
			jlink_clock_data(NULL, 0, &tms, 0, NULL, 0, 1);
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static void jlink_stableclocks(int num_cycles)
{
	int i;

	uint8_t tms = tap_get_state() == TAP_RESET;
	/* Execute num_cycles. */
	for (i = 0; i < num_cycles; i++)
		jlink_clock_data(NULL, 0, &tms, 0, NULL, 0, 1);
}

static void jlink_runtest(int num_cycles)
{
	tap_state_t saved_end_state = tap_get_end_state();

	/* Only do a state_move when we're not already in IDLE. */
	if (tap_get_state() != TAP_IDLE) {
		jlink_end_state(TAP_IDLE);
		jlink_state_move();
		/* num_cycles--; */
	}

	jlink_stableclocks(num_cycles);

	/* Finish in end_state. */
	jlink_end_state(saved_end_state);

	if (tap_get_state() != tap_get_end_state())
		jlink_state_move();
}

static void jlink_reset(int trst, int srst)
{
	LOG_DEBUG("TRST: %i, SRST: %i.", trst, srst);

	/* Signals are active low. */
	if (srst == 0)
		jaylink_set_reset(devh);

	if (srst == 1)
		jaylink_clear_reset(devh);

	if (trst == 1)
		jaylink_jtag_clear_trst(devh);

	if (trst == 0)
		jaylink_jtag_set_trst(devh);
}

static int jlink_reset_safe(int trst, int srst)
{
	jlink_flush();
	jlink_reset(trst, srst);
	return jlink_flush();
}

COMMAND_HANDLER(jlink_usb_command)
{
	int tmp;

	if (CMD_ARGC != 1) {
		command_print(CMD, "Need exactly one argument for jlink usb.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (sscanf(CMD_ARGV[0], "%i", &tmp) != 1) {
		command_print(CMD, "Invalid USB address: %s.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	if (tmp < JAYLINK_USB_ADDRESS_0 || tmp > JAYLINK_USB_ADDRESS_3) {
		command_print(CMD, "Invalid USB address: %s.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	usb_address = tmp;

	use_serial_number = false;
	use_usb_address = true;

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_serial_command)
{
	int ret;

	if (CMD_ARGC != 1) {
		command_print(CMD, "Need exactly one argument for jlink serial.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	ret = jaylink_parse_serial_number(CMD_ARGV[0], &serial_number);

	if (ret == JAYLINK_ERR) {
		command_print(CMD, "Invalid serial number: %s.", CMD_ARGV[0]);
		return ERROR_FAIL;
	} else if (ret != JAYLINK_OK) {
		command_print(CMD, "jaylink_parse_serial_number() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	use_serial_number = true;
	use_usb_address = false;

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_hwstatus_command)
{
	int ret;
	struct jaylink_hardware_status status;

	ret = jaylink_get_hardware_status(devh, &status);

	if (ret != JAYLINK_OK) {
		command_print(CMD, "jaylink_get_hardware_status() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	command_print(CMD, "VTarget = %u.%03u V",
		status.target_voltage / 1000, status.target_voltage % 1000);

	command_print(CMD, "TCK = %u TDI = %u TDO = %u TMS = %u SRST = %u "
		"TRST = %u", status.tck, status.tdi, status.tdo, status.tms,
		status.tres, status.trst);

	if (status.target_voltage < 1500)
		command_print(CMD, "Target voltage too low. Check target power.");

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_free_memory_command)
{
	int ret;
	uint32_t tmp;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_GET_FREE_MEMORY)) {
		command_print(CMD, "Retrieval of free memory is not supported by "
			"the device.");
		return ERROR_OK;
	}

	ret = jaylink_get_free_memory(devh, &tmp);

	if (ret != JAYLINK_OK) {
		command_print(CMD, "jaylink_get_free_memory() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	command_print(CMD, "Device has %" PRIu32 " bytes of free memory.", tmp);

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_jtag_command)
{
	int tmp;
	int version;

	if (!CMD_ARGC) {
		switch (jtag_command_version) {
			case JAYLINK_JTAG_VERSION_2:
				version = 2;
				break;
			case JAYLINK_JTAG_VERSION_3:
				version = 3;
				break;
			default:
				return ERROR_FAIL;
		}

		command_print(CMD, "JTAG command version: %i", version);
	} else if (CMD_ARGC == 1) {
		if (sscanf(CMD_ARGV[0], "%i", &tmp) != 1) {
			command_print(CMD, "Invalid argument: %s.", CMD_ARGV[0]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		switch (tmp) {
			case 2:
				jtag_command_version = JAYLINK_JTAG_VERSION_2;
				break;
			case 3:
				jtag_command_version = JAYLINK_JTAG_VERSION_3;
				break;
			default:
				command_print(CMD, "Invalid argument: %s.", CMD_ARGV[0]);
				return ERROR_COMMAND_SYNTAX_ERROR;
		}
	} else {
		command_print(CMD, "Need exactly one argument for jlink jtag.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_target_power_command)
{
	int ret;
	int enable;

	if (CMD_ARGC != 1) {
		command_print(CMD, "Need exactly one argument for jlink "
			"targetpower.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_SET_TARGET_POWER)) {
		command_print(CMD, "Target power supply is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!strcmp(CMD_ARGV[0], "on")) {
		enable = true;
	} else if (!strcmp(CMD_ARGV[0], "off")) {
		enable = false;
	} else {
		command_print(CMD, "Invalid argument: %s.", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	ret = jaylink_set_target_power(devh, enable);

	if (ret != JAYLINK_OK) {
		command_print(CMD, "jaylink_set_target_power() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static void show_config_usb_address(struct command_invocation *cmd)
{
	if (config.usb_address != tmp_config.usb_address)
		command_print(cmd, "USB address: %u [%u]", config.usb_address,
			tmp_config.usb_address);
	else
		command_print(cmd, "USB address: %u", config.usb_address);
}

static void show_config_ip_address(struct command_invocation *cmd)
{
	if (!memcmp(config.ip_address, tmp_config.ip_address, 4))
		command_print(cmd, "IP address: %d.%d.%d.%d",
			config.ip_address[3], config.ip_address[2],
			config.ip_address[1], config.ip_address[0]);
	else
		command_print(cmd, "IP address: %d.%d.%d.%d [%d.%d.%d.%d]",
			config.ip_address[3], config.ip_address[2],
			config.ip_address[1], config.ip_address[0],
			tmp_config.ip_address[3], tmp_config.ip_address[2],
			tmp_config.ip_address[1], tmp_config.ip_address[0]);

	if (!memcmp(config.subnet_mask, tmp_config.subnet_mask, 4))
		command_print(cmd, "Subnet mask: %d.%d.%d.%d",
			config.subnet_mask[3], config.subnet_mask[2],
			config.subnet_mask[1], config.subnet_mask[0]);
	else
		command_print(cmd, "Subnet mask: %d.%d.%d.%d [%d.%d.%d.%d]",
			config.subnet_mask[3], config.subnet_mask[2],
			config.subnet_mask[1], config.subnet_mask[0],
			tmp_config.subnet_mask[3], tmp_config.subnet_mask[2],
			tmp_config.subnet_mask[1], tmp_config.subnet_mask[0]);
}

static void show_config_mac_address(struct command_invocation *cmd)
{
	if (!memcmp(config.mac_address, tmp_config.mac_address, 6))
		command_print(cmd, "MAC address: %.02x:%.02x:%.02x:%.02x:%.02x:%.02x",
			config.mac_address[5], config.mac_address[4],
			config.mac_address[3], config.mac_address[2],
			config.mac_address[1], config.mac_address[0]);
	else
		command_print(cmd, "MAC address: %.02x:%.02x:%.02x:%.02x:%.02x:%.02x "
			"[%.02x:%.02x:%.02x:%.02x:%.02x:%.02x]",
			config.mac_address[5], config.mac_address[4],
			config.mac_address[3], config.mac_address[2],
			config.mac_address[1], config.mac_address[0],
			tmp_config.mac_address[5], tmp_config.mac_address[4],
			tmp_config.mac_address[3], tmp_config.mac_address[2],
			tmp_config.mac_address[1], tmp_config.mac_address[0]);
}

static void show_config_target_power(struct command_invocation *cmd)
{
	const char *target_power;
	const char *current_target_power;

	if (!config.target_power)
		target_power = "off";
	else
		target_power = "on";

	if (!tmp_config.target_power)
		current_target_power = "off";
	else
		current_target_power = "on";

	if (config.target_power != tmp_config.target_power)
		command_print(cmd, "Target power supply: %s [%s]", target_power,
			current_target_power);
	else
		command_print(cmd, "Target power supply: %s", target_power);
}

static void show_config(struct command_invocation *cmd)
{
	command_print(cmd, "J-Link device configuration:");

	show_config_usb_address(cmd);

	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_SET_TARGET_POWER))
		show_config_target_power(cmd);

	if (jaylink_has_cap(caps, JAYLINK_DEV_CAP_ETHERNET)) {
		show_config_ip_address(cmd);
		show_config_mac_address(cmd);
	}
}

static int poll_trace(uint8_t *buf, size_t *size)
{
	int ret;
	uint32_t length;

	length = *size;

	ret = jaylink_swo_read(devh, buf, &length);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_swo_read() failed: %s.", jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	*size = length;

	return ERROR_OK;
}

static uint32_t calculate_trace_buffer_size(void)
{
	int ret;
	uint32_t tmp;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_GET_FREE_MEMORY))
		return 0;

	ret = jaylink_get_free_memory(devh, &tmp);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_get_free_memory() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	if (tmp > 0x3fff || tmp <= 0x600)
		tmp = tmp >> 1;
	else
		tmp = tmp - 0x400;

	return tmp & 0xffffff00;
}

static bool calculate_swo_prescaler(unsigned int traceclkin_freq,
		uint32_t trace_freq, uint16_t *prescaler)
{
	unsigned int presc = (traceclkin_freq + trace_freq / 2) / trace_freq;
	if (presc == 0 || presc > TPIU_ACPR_MAX_SWOSCALER + 1)
		return false;

	/* Probe's UART speed must be within 3% of the TPIU's SWO baud rate. */
	unsigned int max_deviation = (traceclkin_freq * 3) / 100;
	if (presc * trace_freq < traceclkin_freq - max_deviation ||
	    presc * trace_freq > traceclkin_freq + max_deviation)
		return false;

	*prescaler = presc;

	return true;
}

static bool detect_swo_freq_and_prescaler(struct jaylink_swo_speed speed,
		unsigned int traceclkin_freq, unsigned int *trace_freq,
		uint16_t *prescaler)
{
	uint32_t divider;
	unsigned int presc;
	double deviation;

	for (divider = speed.min_div; divider <= speed.max_div; divider++) {
		*trace_freq = speed.freq / divider;
		presc = ((1.0 - SWO_MAX_FREQ_DEV) * traceclkin_freq) / *trace_freq + 1;

		if (presc > TPIU_ACPR_MAX_SWOSCALER + 1)
			break;

		deviation = fabs(1.0 - ((double)*trace_freq * presc / traceclkin_freq));

		if (deviation <= SWO_MAX_FREQ_DEV) {
			*prescaler = presc;
			return true;
		}
	}

	return false;
}

static int config_trace(bool enabled, enum tpiu_pin_protocol pin_protocol,
		uint32_t port_size, unsigned int *trace_freq,
		unsigned int traceclkin_freq, uint16_t *prescaler)
{
	int ret;
	uint32_t buffer_size;
	struct jaylink_swo_speed speed;
	uint32_t divider;
	uint32_t min_freq;
	uint32_t max_freq;

	trace_enabled = enabled;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_SWO)) {
		if (!enabled)
			return ERROR_OK;

		LOG_ERROR("Trace capturing is not supported by the device.");
		return ERROR_FAIL;
	}

	ret = jaylink_swo_stop(devh);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_swo_stop() failed: %s.", jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	if (!enabled) {
		/*
		 * Adjust the SWD transaction buffer size as stopping SWO capturing
		 * deallocates device internal memory.
		 */
		if (!adjust_swd_buffer_size())
			return ERROR_FAIL;

		return ERROR_OK;
	}

	if (pin_protocol != TPIU_PIN_PROTOCOL_ASYNC_UART) {
		LOG_ERROR("Selected pin protocol is not supported.");
		return ERROR_FAIL;
	}

	buffer_size = calculate_trace_buffer_size();

	if (!buffer_size) {
		LOG_ERROR("Not enough free device memory to start trace capturing.");
		return ERROR_FAIL;
	}

	ret = jaylink_swo_get_speeds(devh, JAYLINK_SWO_MODE_UART, &speed);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_swo_get_speeds() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	if (*trace_freq > 0) {
		divider = speed.freq / *trace_freq;
		min_freq = speed.freq / speed.max_div;
		max_freq = speed.freq / speed.min_div;

		if (*trace_freq > max_freq) {
			LOG_INFO("Given SWO frequency too high, using %" PRIu32 " Hz instead.",
				max_freq);
			*trace_freq = max_freq;
		} else if (*trace_freq < min_freq) {
			LOG_INFO("Given SWO frequency too low, using %" PRIu32 " Hz instead.",
				min_freq);
			*trace_freq = min_freq;
		} else if (*trace_freq != speed.freq / divider) {
			*trace_freq = speed.freq / divider;

			LOG_INFO("Given SWO frequency is not supported by the device, "
				"using %u Hz instead.", *trace_freq);
		}

		if (!calculate_swo_prescaler(traceclkin_freq, *trace_freq,
				prescaler)) {
			LOG_ERROR("SWO frequency is not suitable. Please choose a "
				"different frequency or use auto-detection.");
			return ERROR_FAIL;
		}
	} else {
		LOG_INFO("Trying to auto-detect SWO frequency.");

		if (!detect_swo_freq_and_prescaler(speed, traceclkin_freq, trace_freq,
				prescaler)) {
			LOG_ERROR("Maximum permitted frequency deviation of %.02f %% "
				"could not be achieved.", SWO_MAX_FREQ_DEV);
			LOG_ERROR("Auto-detection of SWO frequency failed.");
			return ERROR_FAIL;
		}

		LOG_INFO("Using SWO frequency of %u Hz.", *trace_freq);
	}

	ret = jaylink_swo_start(devh, JAYLINK_SWO_MODE_UART, *trace_freq,
		buffer_size);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_start_swo() failed: %s.", jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	LOG_DEBUG("Using %" PRIu32 " bytes device memory for trace capturing.",
		buffer_size);

	/*
	 * Adjust the SWD transaction buffer size as starting SWO capturing
	 * allocates device internal memory.
	 */
	if (!adjust_swd_buffer_size())
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_config_usb_address_command)
{
	uint8_t tmp;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_READ_CONFIG)) {
		command_print(CMD, "Reading configuration is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!CMD_ARGC) {
		show_config_usb_address(CMD);
	} else if (CMD_ARGC == 1) {
		if (sscanf(CMD_ARGV[0], "%" SCNd8, &tmp) != 1) {
			command_print(CMD, "Invalid USB address: %s.", CMD_ARGV[0]);
			return ERROR_FAIL;
		}

		if (tmp > JAYLINK_USB_ADDRESS_3) {
			command_print(CMD, "Invalid USB address: %u.", tmp);
			return ERROR_FAIL;
		}

		tmp_config.usb_address = tmp;
	} else {
		command_print(CMD, "Need exactly one argument for jlink config "
			"usb.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_config_target_power_command)
{
	int enable;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_READ_CONFIG)) {
		command_print(CMD, "Reading configuration is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_SET_TARGET_POWER)) {
		command_print(CMD, "Target power supply is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!CMD_ARGC) {
		show_config_target_power(CMD);
	} else if (CMD_ARGC == 1) {
		if (!strcmp(CMD_ARGV[0], "on")) {
			enable = true;
		} else if (!strcmp(CMD_ARGV[0], "off")) {
			enable = false;
		} else {
			command_print(CMD, "Invalid argument: %s.", CMD_ARGV[0]);
			return ERROR_FAIL;
		}

		tmp_config.target_power = enable;
	} else {
		command_print(CMD, "Need exactly one argument for jlink config "
			"targetpower.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_config_mac_address_command)
{
	uint8_t addr[6];
	int i;
	char *e;
	const char *str;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_READ_CONFIG)) {
		command_print(CMD, "Reading configuration is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_ETHERNET)) {
		command_print(CMD, "Ethernet connectivity is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!CMD_ARGC) {
		show_config_mac_address(CMD);
	} else if (CMD_ARGC == 1) {
		str = CMD_ARGV[0];

		if ((strlen(str) != 17) || (str[2] != ':' || str[5] != ':' ||
				str[8] != ':' || str[11] != ':' || str[14] != ':')) {
			command_print(CMD, "Invalid MAC address format.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		for (i = 5; i >= 0; i--) {
			addr[i] = strtoul(str, &e, 16);
			str = e + 1;
		}

		if (!(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5])) {
			command_print(CMD, "Invalid MAC address: zero address.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (!(0x01 & addr[0])) {
			command_print(CMD, "Invalid MAC address: multicast address.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		memcpy(tmp_config.mac_address, addr, sizeof(addr));
	} else {
		command_print(CMD, "Need exactly one argument for jlink config "
			" mac.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static bool string_to_ip(const char *s, uint8_t *ip, int *pos)
{
	uint8_t lip[4];
	char *e;
	const char *s_save = s;
	int i;

	if (!s)
		return false;

	for (i = 0; i < 4; i++) {
		lip[i] = strtoul(s, &e, 10);

		if (*e != '.' && i != 3)
			return false;

		s = e + 1;
	}

	*pos = e - s_save;
	memcpy(ip, lip, sizeof(lip));

	return true;
}

static void cpy_ip(uint8_t *dst, uint8_t *src)
{
	int i, j;

	for (i = 0, j = 3; i < 4; i++, j--)
		dst[i] = src[j];
}

COMMAND_HANDLER(jlink_handle_config_ip_address_command)
{
	uint8_t ip_address[4];
	uint32_t subnet_mask = 0;
	int i, len;
	uint8_t subnet_bits = 24;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_READ_CONFIG)) {
		command_print(CMD, "Reading configuration is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_ETHERNET)) {
		command_print(CMD, "Ethernet connectivity is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!CMD_ARGC) {
		show_config_ip_address(CMD);
	} else {
		if (!string_to_ip(CMD_ARGV[0], ip_address, &i))
			return ERROR_COMMAND_SYNTAX_ERROR;

		len = strlen(CMD_ARGV[0]);

		/* Check for format A.B.C.D/E. */
		if (i < len) {
			if (CMD_ARGV[0][i] != '/')
				return ERROR_COMMAND_SYNTAX_ERROR;

			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0] + i + 1, subnet_bits);
		} else if (CMD_ARGC > 1) {
			if (!string_to_ip(CMD_ARGV[1], (uint8_t *)&subnet_mask, &i))
				return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (!subnet_mask)
			subnet_mask = (uint32_t)(subnet_bits < 32 ?
				((1ULL << subnet_bits) - 1) : 0xffffffff);

		cpy_ip(tmp_config.ip_address, ip_address);
		cpy_ip(tmp_config.subnet_mask, (uint8_t *)&subnet_mask);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_config_reset_command)
{
	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_READ_CONFIG))
		return ERROR_OK;

	memcpy(&tmp_config, &config, sizeof(struct device_config));

	return ERROR_OK;
}


COMMAND_HANDLER(jlink_handle_config_write_command)
{
	int ret;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_READ_CONFIG)) {
		command_print(CMD, "Reading configuration is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_WRITE_CONFIG)) {
		command_print(CMD, "Writing configuration is not supported by the "
			"device.");
		return ERROR_OK;
	}

	if (!memcmp(&config, &tmp_config, sizeof(struct device_config))) {
		command_print(CMD, "Operation not performed due to no changes in "
			"the configuration.");
		return ERROR_OK;
	}

	ret = jaylink_write_raw_config(devh, (const uint8_t *)&tmp_config);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_write_raw_config() failed: %s.",
			jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	if (!read_device_config(&config)) {
		LOG_ERROR("Failed to read device configuration for verification.");
		return ERROR_FAIL;
	}

	if (memcmp(&config, &tmp_config, sizeof(struct device_config))) {
		LOG_ERROR("Verification of device configuration failed. Please check "
			"your device.");
		return ERROR_FAIL;
	}

	memcpy(&tmp_config, &config, sizeof(struct device_config));
	command_print(CMD, "The new device configuration applies after power "
		"cycling the J-Link device.");

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_config_command)
{
	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_READ_CONFIG)) {
		command_print(CMD, "Device doesn't support reading configuration.");
		return ERROR_OK;
	}

	if (CMD_ARGC == 0)
		show_config(CMD);

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_emucom_write_command)
{
	int ret;
	size_t tmp;
	uint32_t channel;
	uint32_t length;
	uint8_t *buf;
	size_t dummy;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_EMUCOM)) {
		LOG_ERROR("Device does not support EMUCOM.");
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], channel);

	tmp = strlen(CMD_ARGV[1]);

	if (tmp % 2 != 0) {
		LOG_ERROR("Data must be encoded as hexadecimal pairs.");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	buf = malloc(tmp / 2);

	if (!buf) {
		LOG_ERROR("Failed to allocate buffer.");
		return ERROR_FAIL;
	}

	dummy = unhexify(buf, CMD_ARGV[1], tmp / 2);

	if (dummy != (tmp / 2)) {
		LOG_ERROR("Data must be encoded as hexadecimal pairs.");
		free(buf);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	length = tmp / 2;
	ret = jaylink_emucom_write(devh, channel, buf, &length);

	free(buf);

	if (ret == JAYLINK_ERR_DEV_NOT_SUPPORTED) {
		LOG_ERROR("Channel not supported by the device.");
		return ERROR_FAIL;
	} else if (ret != JAYLINK_OK) {
		LOG_ERROR("Failed to write to channel: %s.", jaylink_strerror(ret));
		return ERROR_FAIL;
	}

	if (length != (tmp / 2))
		LOG_WARNING("Only %" PRIu32 " bytes written to the channel.", length);

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_emucom_read_command)
{
	int ret;
	uint32_t channel;
	uint32_t length;
	uint8_t *buf;
	size_t tmp;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!jaylink_has_cap(caps, JAYLINK_DEV_CAP_EMUCOM)) {
		LOG_ERROR("Device does not support EMUCOM.");
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], channel);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], length);

	buf = malloc(length * 3 + 1);

	if (!buf) {
		LOG_ERROR("Failed to allocate buffer.");
		return ERROR_FAIL;
	}

	ret = jaylink_emucom_read(devh, channel, buf, &length);

	if (ret == JAYLINK_ERR_DEV_NOT_SUPPORTED) {
		LOG_ERROR("Channel is not supported by the device.");
		free(buf);
		return ERROR_FAIL;
	} else if (ret == JAYLINK_ERR_DEV_NOT_AVAILABLE) {
		LOG_ERROR("Channel is not available for the requested amount of data. "
			"%" PRIu32 " bytes are available.", length);
		free(buf);
		return ERROR_FAIL;
	} else if (ret != JAYLINK_OK) {
		LOG_ERROR("Failed to read from channel: %s.", jaylink_strerror(ret));
		free(buf);
		return ERROR_FAIL;
	}

	tmp = hexify((char *)buf + length, buf, length, 2 * length + 1);

	if (tmp != 2 * length) {
		LOG_ERROR("Failed to convert data into hexadecimal string.");
		free(buf);
		return ERROR_FAIL;
	}

	command_print(CMD, "%s", buf + length);
	free(buf);

	return ERROR_OK;
}

static const struct command_registration jlink_config_subcommand_handlers[] = {
	{
		.name = "usb",
		.handler = &jlink_handle_config_usb_address_command,
		.mode = COMMAND_EXEC,
		.help = "set the USB address",
		.usage = "[0-3]",
	},
	{
		.name = "targetpower",
		.handler = &jlink_handle_config_target_power_command,
		.mode = COMMAND_EXEC,
		.help = "set the target power supply",
		.usage = "[on|off]"
	},
	{
		.name = "mac",
		.handler = &jlink_handle_config_mac_address_command,
		.mode = COMMAND_EXEC,
		.help = "set the MAC Address",
		.usage = "[ff:ff:ff:ff:ff:ff]",
	},
	{
		.name = "ip",
		.handler = &jlink_handle_config_ip_address_command,
		.mode = COMMAND_EXEC,
		.help = "set the IP address, where A.B.C.D is the IP address, "
			"E the bit of the subnet mask, F.G.H.I the subnet mask",
		.usage = "[A.B.C.D[/E] [F.G.H.I]]",
	},
	{
		.name = "reset",
		.handler = &jlink_handle_config_reset_command,
		.mode = COMMAND_EXEC,
		.help = "undo configuration changes",
		.usage = "",
	},
	{
		.name = "write",
		.handler = &jlink_handle_config_write_command,
		.mode = COMMAND_EXEC,
		.help = "write configuration to the device",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration jlink_emucom_subcommand_handlers[] = {
	{
		.name = "write",
		.handler = &jlink_handle_emucom_write_command,
		.mode = COMMAND_EXEC,
		.help = "write to a channel",
		.usage = "<channel> <data>",
	},
	{
		.name = "read",
		.handler = &jlink_handle_emucom_read_command,
		.mode = COMMAND_EXEC,
		.help = "read from a channel",
		.usage = "<channel> <length>"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration jlink_subcommand_handlers[] = {
	{
		.name = "jtag",
		.handler = &jlink_handle_jlink_jtag_command,
		.mode = COMMAND_EXEC,
		.help = "select the JTAG command version",
		.usage = "[2|3]",
	},
	{
		.name = "targetpower",
		.handler = &jlink_handle_target_power_command,
		.mode = COMMAND_EXEC,
		.help = "set the target power supply",
		.usage = "<on|off>"
	},
	{
		.name = "freemem",
		.handler = &jlink_handle_free_memory_command,
		.mode = COMMAND_EXEC,
		.help = "show free device memory",
		.usage = "",
	},
	{
		.name = "hwstatus",
		.handler = &jlink_handle_hwstatus_command,
		.mode = COMMAND_EXEC,
		.help = "show the hardware status",
		.usage = "",
	},
	{
		.name = "usb",
		.handler = &jlink_usb_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB address of the device that should be used",
		.usage = "<0-3>"
	},
	{
		.name = "serial",
		.handler = &jlink_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the device that should be used",
		.usage = "<serial number>"
	},
	{
		.name = "config",
		.handler = &jlink_handle_config_command,
		.mode = COMMAND_EXEC,
		.help = "access the device configuration. If no argument is given "
			"this will show the device configuration",
		.chain = jlink_config_subcommand_handlers,
		.usage = "[<cmd>]",
	},
	{
		.name = "emucom",
		.mode = COMMAND_EXEC,
		.help = "access EMUCOM channel",
		.chain = jlink_emucom_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration jlink_command_handlers[] = {
	{
		.name = "jlink",
		.mode = COMMAND_ANY,
		.help = "perform jlink management",
		.chain = jlink_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int jlink_swd_init(void)
{
	iface = JAYLINK_TIF_SWD;

	return ERROR_OK;
}

static void jlink_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	jlink_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static void jlink_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	jlink_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

/***************************************************************************/
/* J-Link tap functions */

static unsigned tap_length;
/* In SWD mode use tms buffer for direction control */
static uint8_t tms_buffer[JLINK_TAP_BUFFER_SIZE];
static uint8_t tdi_buffer[JLINK_TAP_BUFFER_SIZE];
static uint8_t tdo_buffer[JLINK_TAP_BUFFER_SIZE];

struct pending_scan_result {
	/** First bit position in tdo_buffer to read. */
	unsigned first;
	/** Number of bits to read. */
	unsigned length;
	/** Location to store the result */
	void *buffer;
	/** Offset in the destination buffer */
	unsigned buffer_offset;
};

#define MAX_PENDING_SCAN_RESULTS 256

static int pending_scan_results_length;
static struct pending_scan_result pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

static void jlink_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
	memset(tms_buffer, 0, sizeof(tms_buffer));
	memset(tdi_buffer, 0, sizeof(tdi_buffer));
}

static void jlink_clock_data(const uint8_t *out, unsigned out_offset,
			     const uint8_t *tms_out, unsigned tms_offset,
			     uint8_t *in, unsigned in_offset,
			     unsigned length)
{
	do {
		unsigned available_length = JLINK_TAP_BUFFER_SIZE - tap_length / 8;

		if (!available_length ||
		    (in && pending_scan_results_length == MAX_PENDING_SCAN_RESULTS)) {
			if (jlink_flush() != ERROR_OK)
				return;
			available_length = JLINK_TAP_BUFFER_SIZE;
		}

		struct pending_scan_result *pending_scan_result =
			&pending_scan_results_buffer[pending_scan_results_length];

		unsigned scan_length = length > available_length ?
			available_length : length;

		if (out)
			buf_set_buf(out, out_offset, tdi_buffer, tap_length, scan_length);
		if (tms_out)
			buf_set_buf(tms_out, tms_offset, tms_buffer, tap_length, scan_length);

		if (in) {
			pending_scan_result->first = tap_length;
			pending_scan_result->length = scan_length;
			pending_scan_result->buffer = in;
			pending_scan_result->buffer_offset = in_offset;
			pending_scan_results_length++;
		}

		tap_length += scan_length;
		out_offset += scan_length;
		tms_offset += scan_length;
		in_offset += scan_length;
		length -= scan_length;
	} while (length > 0);
}

static int jlink_flush(void)
{
	int i;
	int ret;

	if (!tap_length)
		return ERROR_OK;

	jlink_last_state = jtag_debug_state_machine(tms_buffer, tdi_buffer,
		tap_length, jlink_last_state);

	ret = jaylink_jtag_io(devh, tms_buffer, tdi_buffer, tdo_buffer,
		tap_length, jtag_command_version);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_jtag_io() failed: %s.", jaylink_strerror(ret));
		jlink_tap_init();
		return ERROR_JTAG_QUEUE_FAILED;
	}

	for (i = 0; i < pending_scan_results_length; i++) {
		struct pending_scan_result *p = &pending_scan_results_buffer[i];

		buf_set_buf(tdo_buffer, p->first, p->buffer,
			    p->buffer_offset, p->length);

		LOG_DEBUG_IO("Pending scan result, length = %d.", p->length);
	}

	jlink_tap_init();

	return ERROR_OK;
}

static void fill_buffer(uint8_t *buf, uint32_t val, uint32_t len)
{
	unsigned int tap_pos = tap_length;

	while (len > 32) {
		buf_set_u32(buf, tap_pos, 32, val);
		len -= 32;
		tap_pos += 32;
	}

	if (len)
		buf_set_u32(buf, tap_pos, len, val);
}

static void jlink_queue_data_out(const uint8_t *data, uint32_t len)
{
	const uint32_t dir_out = 0xffffffff;

	if (data)
		bit_copy(tdi_buffer, tap_length, data, 0, len);
	else
		fill_buffer(tdi_buffer, 0, len);

	fill_buffer(tms_buffer, dir_out, len);
	tap_length += len;
}

static void jlink_queue_data_in(uint32_t len)
{
	const uint32_t dir_in = 0;

	fill_buffer(tms_buffer, dir_in, len);
	tap_length += len;
}

static int jlink_swd_switch_seq(enum swd_special_seq seq)
{
	const uint8_t *s;
	unsigned int s_len;

	switch (seq) {
		case LINE_RESET:
			LOG_DEBUG("SWD line reset");
			s = swd_seq_line_reset;
			s_len = swd_seq_line_reset_len;
			break;
		case JTAG_TO_SWD:
			LOG_DEBUG("JTAG-to-SWD");
			s = swd_seq_jtag_to_swd;
			s_len = swd_seq_jtag_to_swd_len;
			break;
		case SWD_TO_JTAG:
			LOG_DEBUG("SWD-to-JTAG");
			s = swd_seq_swd_to_jtag;
			s_len = swd_seq_swd_to_jtag_len;
			break;
		default:
			LOG_ERROR("Sequence %d not supported.", seq);
			return ERROR_FAIL;
	}

	jlink_queue_data_out(s, s_len);

	return ERROR_OK;
}

static int jlink_swd_run_queue(void)
{
	int i;
	int ret;

	LOG_DEBUG("Executing %d queued transactions.", pending_scan_results_length);

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skipping due to previous errors: %d.", queued_retval);
		goto skip;
	}

	/*
	 * A transaction must be followed by another transaction or at least 8 idle
	 * cycles to ensure that data is clocked through the AP.
	 */
	jlink_queue_data_out(NULL, 8);

	ret = jaylink_swd_io(devh, tms_buffer, tdi_buffer, tdo_buffer, tap_length);

	if (ret != JAYLINK_OK) {
		LOG_ERROR("jaylink_swd_io() failed: %s.", jaylink_strerror(ret));
		goto skip;
	}

	for (i = 0; i < pending_scan_results_length; i++) {
		int ack = buf_get_u32(tdo_buffer, pending_scan_results_buffer[i].first, 3);

		if (ack != SWD_ACK_OK) {
			LOG_DEBUG("SWD ack not OK: %d %s", ack,
				  ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK");
			queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
			goto skip;
		} else if (pending_scan_results_buffer[i].length) {
			uint32_t data = buf_get_u32(tdo_buffer, 3 + pending_scan_results_buffer[i].first, 32);
			int parity = buf_get_u32(tdo_buffer, 3 + 32 + pending_scan_results_buffer[i].first, 1);

			if (parity != parity_u32(data)) {
				LOG_ERROR("SWD: Read data parity mismatch.");
				queued_retval = ERROR_FAIL;
				goto skip;
			}

			if (pending_scan_results_buffer[i].buffer)
				*(uint32_t *)pending_scan_results_buffer[i].buffer = data;
		}
	}

skip:
	jlink_tap_init();
	ret = queued_retval;
	queued_retval = ERROR_OK;

	return ret;
}

static void jlink_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	uint8_t data_parity_trn[DIV_ROUND_UP(32 + 1, 8)];
	if (tap_length + 46 + 8 + ap_delay_clk >= swd_buffer_size * 8 ||
	    pending_scan_results_length == MAX_PENDING_SCAN_RESULTS) {
		/* Not enough room in the queue. Run the queue. */
		queued_retval = jlink_swd_run_queue();
	}

	if (queued_retval != ERROR_OK)
		return;

	cmd |= SWD_CMD_START | SWD_CMD_PARK;

	jlink_queue_data_out(&cmd, 8);

	pending_scan_results_buffer[pending_scan_results_length].first = tap_length;

	if (cmd & SWD_CMD_RnW) {
		/* Queue a read transaction. */
		pending_scan_results_buffer[pending_scan_results_length].length = 32;
		pending_scan_results_buffer[pending_scan_results_length].buffer = dst;

		jlink_queue_data_in(1 + 3 + 32 + 1 + 1);
	} else {
		/* Queue a write transaction. */
		pending_scan_results_buffer[pending_scan_results_length].length = 0;
		jlink_queue_data_in(1 + 3 + 1);

		buf_set_u32(data_parity_trn, 0, 32, data);
		buf_set_u32(data_parity_trn, 32, 1, parity_u32(data));

		jlink_queue_data_out(data_parity_trn, 32 + 1);
	}

	pending_scan_results_length++;

	/* Insert idle cycles after AP accesses to avoid WAIT. */
	if (cmd & SWD_CMD_APnDP)
		jlink_queue_data_out(NULL, ap_delay_clk);
}

static const struct swd_driver jlink_swd = {
	.init = &jlink_swd_init,
	.switch_seq = &jlink_swd_switch_seq,
	.read_reg = &jlink_swd_read_reg,
	.write_reg = &jlink_swd_write_reg,
	.run = &jlink_swd_run_queue,
};

static const char * const jlink_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface jlink_interface = {
	.execute_queue = &jlink_execute_queue,
};

struct adapter_driver jlink_adapter_driver = {
	.name = "jlink",
	.transports = jlink_transports,
	.commands = jlink_command_handlers,

	.init = &jlink_init,
	.quit = &jlink_quit,
	.reset = &jlink_reset_safe,
	.speed = &jlink_speed,
	.khz = &jlink_khz,
	.speed_div = &jlink_speed_div,
	.config_trace = &config_trace,
	.poll_trace = &poll_trace,

	.jtag_ops = &jlink_interface,
	.swd_ops = &jlink_swd,
};
