/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2019 Google LLC.
 * Author: Moritz Fischer <moritzf@google.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <linux/pci.h>

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>
#include <helper/replacements.h>
#include <helper/bits.h>

#define PCIE_EXT_CAP_LST	0x100

#define XLNX_XVC_EXT_CAP	0x00
#define XLNX_XVC_VSEC_HDR	0x04
#define XLNX_XVC_VERSION	0x08
#define XLNX_XVC_LEN_REG	0x0C
#define XLNX_XVC_TMS_REG	0x10
#define XLNX_XVC_TDx_REG	0x14
#define XLNX_XVC_STS_REG	0x1c

#define XLNX_XVC_CAP_SIZE	0x20
#define XLNX_XVC_VSEC_ID	0x8
#define XLNX_XVC_MAX_BITS	0x20

struct xlnx_pcie_xvc {
	int fd;
	int offset;
	char *device;
};

static struct xlnx_pcie_xvc xlnx_pcie_xvc_state;
static struct xlnx_pcie_xvc *xlnx_pcie_xvc = &xlnx_pcie_xvc_state;

static uint32_t xlnx_pcie_xvc_read_reg(const int offset)
{
	uint32_t res = 0xffffffff;
	int ret;

	/* Note: This should be ok endianess-wise because by going
	 * through sysfs the kernel does the conversion in the config
	 * space accessor functions
	 */
	ret = pread(xlnx_pcie_xvc->fd, &res, sizeof(res),
		    xlnx_pcie_xvc->offset + offset);
	if (ret != sizeof(res))
		LOG_ERROR("Failed to read offset %x", offset);

	return res;
}

static void xlnx_pcie_xvc_write_reg(const int offset, const uint32_t val)
{
	int ret;

	/* Note: This should be ok endianess-wise because by going
	 * through sysfs the kernel does the conversion in the config
	 * space accessor functions
	 */
	ret = pwrite(xlnx_pcie_xvc->fd, &val, sizeof(val),
		     xlnx_pcie_xvc->offset + offset);
	if (ret != sizeof(val))
		LOG_ERROR("Failed to write offset: %x with value: %x",
			  offset, val);
}

static void xlnx_pcie_xvc_transact(size_t num_bits, uint32_t tms, uint32_t tdi,
				   uint32_t *tdo)
{
	xlnx_pcie_xvc_write_reg(XLNX_XVC_LEN_REG, num_bits);
	xlnx_pcie_xvc_write_reg(XLNX_XVC_TMS_REG, tms);
	xlnx_pcie_xvc_write_reg(XLNX_XVC_TDx_REG, tdi);
	if (tdo) {
		*tdo = xlnx_pcie_xvc_read_reg(XLNX_XVC_TDx_REG);
		LOG_DEBUG_IO("Transact num_bits: %zu, tms: %x, tdi: %x, tdo: %x",
			     num_bits, tms, tdi, *tdo);
	} else {
		(void)xlnx_pcie_xvc_read_reg(XLNX_XVC_TDx_REG);
		LOG_DEBUG("Transact num_bits: %zu, tms: %x, tdi: %x, tdo: <null>",
			  num_bits, tms, tdi);
	}
}

void xlnx_pcie_xvc_execute_stableclocks(struct jtag_command *cmd)
{
	int tms = tap_get_state() == TAP_RESET ? 1 : 0;
	size_t left = cmd->cmd.stableclocks->num_cycles;
	size_t write;

	LOG_DEBUG("stableclocks %i cycles", cmd->cmd.runtest->num_cycles);

	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		xlnx_pcie_xvc_transact(write, tms, 0, NULL);
		left -= write;
	};
}

static void xlnx_pcie_xvc_execute_statemove(size_t skip)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(),
					    tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(),
					     tap_get_end_state());

	LOG_DEBUG("statemove starting at (skip: %zu) %s end in %s", skip,
		  tap_state_name(tap_get_state()),
		  tap_state_name(tap_get_end_state()));


	xlnx_pcie_xvc_transact(tms_count - skip, tms_scan >> skip, 0, NULL);
	tap_set_state(tap_get_end_state());
}

static void xlnx_pcie_xvc_execute_runtest(struct jtag_command *cmd)
{
	LOG_DEBUG("runtest %i cycles, end in %i",
		  cmd->cmd.runtest->num_cycles,
		  cmd->cmd.runtest->end_state);

	tap_state_t tmp_state = tap_get_end_state();

	if (tap_get_state() != TAP_IDLE) {
		tap_set_end_state(TAP_IDLE);
		xlnx_pcie_xvc_execute_statemove(0);
	};

	size_t left = cmd->cmd.runtest->num_cycles;
	size_t write;

	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		xlnx_pcie_xvc_transact(write, 0, 0, NULL);
		left -= write;
	};

	tap_set_end_state(tmp_state);
	if (tap_get_state() != tap_get_end_state())
		xlnx_pcie_xvc_execute_statemove(0);
}

static void xlnx_pcie_xvc_execute_pathmove(struct jtag_command *cmd)
{
	size_t num_states = cmd->cmd.pathmove->num_states;
	tap_state_t *path = cmd->cmd.pathmove->path;
	size_t i;

	LOG_DEBUG("pathmove: %i states, end in %i",
		  cmd->cmd.pathmove->num_states,
		  cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	for (i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false))
			xlnx_pcie_xvc_transact(1, 1, 0, NULL);
		else if (path[i] == tap_state_transition(tap_get_state(), true))
			xlnx_pcie_xvc_transact(1, 0, 0, NULL);
		else
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				  tap_state_name(tap_get_state()),
				  tap_state_name(path[i]));
		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static int xlnx_pcie_xvc_execute_scan(struct jtag_command *cmd)
{
	enum scan_type type = jtag_scan_type(cmd->cmd.scan);
	tap_state_t saved_end_state = cmd->cmd.scan->end_state;
	bool ir_scan = cmd->cmd.scan->ir_scan;
	uint32_t tdi, tms, tdo;
	uint8_t *buf, *rd_ptr;
	int err, scan_size;
	size_t write;
	size_t left;

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buf);
	rd_ptr = buf;
	LOG_DEBUG("%s scan type %d %d bits; starts in %s end in %s",
		  (cmd->cmd.scan->ir_scan) ? "IR" : "DR", type, scan_size,
		  tap_state_name(tap_get_state()),
		  tap_state_name(cmd->cmd.scan->end_state));

	/* If we're in TAP_DR_SHIFT state but need to do a IR_SCAN or
	 * vice-versa, do a statemove to corresponding other state, then restore
	 * end state
	 */
	if (ir_scan && tap_get_state() != TAP_IRSHIFT) {
		tap_set_end_state(TAP_IRSHIFT);
		xlnx_pcie_xvc_execute_statemove(0);
		tap_set_end_state(saved_end_state);
	} else if (!ir_scan && (tap_get_state() != TAP_DRSHIFT)) {
		tap_set_end_state(TAP_DRSHIFT);
		xlnx_pcie_xvc_execute_statemove(0);
		tap_set_end_state(saved_end_state);
	}

	left = scan_size;
	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		/* the last TMS should be a 1, to leave the state */
		tms = left <= XLNX_XVC_MAX_BITS ? BIT(write - 1) : 0;
		tdi = (type != SCAN_IN) ? buf_get_u32(rd_ptr, 0, write) : 0;
		xlnx_pcie_xvc_transact(write, tms, tdi, type != SCAN_OUT ?
				  &tdo : NULL);
		left -= write;
		buf_set_u32(rd_ptr, 0, write, tdo);
		rd_ptr += sizeof(uint32_t);
	};

	err = jtag_read_buffer(buf, cmd->cmd.scan);
	if (buf)
		free(buf);

	if (tap_get_state() != tap_get_end_state())
		xlnx_pcie_xvc_execute_statemove(1);

	return err;
}

static void xlnx_pcie_xvc_execute_reset(struct jtag_command *cmd)
{
	LOG_DEBUG("reset trst: %i srst: %i", cmd->cmd.reset->trst,
		  cmd->cmd.reset->srst);
}

static void xlnx_pcie_xvc_execute_sleep(struct jtag_command *cmd)
{
	LOG_DEBUG("sleep %" PRIi32 "", cmd->cmd.sleep->us);
	usleep(cmd->cmd.sleep->us);
}

static int xlnx_pcie_xvc_execute_tms(struct jtag_command *cmd)
{
	const size_t num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	size_t left, write;
	uint32_t tms;

	LOG_DEBUG("execute tms %zu", num_bits);

	left = num_bits;
	while (left) {
		write = MIN(XLNX_XVC_MAX_BITS, left);
		tms = buf_get_u32(bits, 0, write);
		xlnx_pcie_xvc_transact(write, tms, 0, NULL);
		left -= write;
		bits += 4;
	};

	return ERROR_OK;
}

static int xlnx_pcie_xvc_execute_command(struct jtag_command *cmd)
{
	LOG_DEBUG("%s: cmd->type: %u", __func__, cmd->type);
	switch (cmd->type) {
	case JTAG_STABLECLOCKS:
		xlnx_pcie_xvc_execute_stableclocks(cmd);
		break;
	case JTAG_RUNTEST:
		xlnx_pcie_xvc_execute_runtest(cmd);
		break;
	case JTAG_TLR_RESET:
		tap_set_end_state(cmd->cmd.statemove->end_state);
		xlnx_pcie_xvc_execute_statemove(0);
		break;
	case JTAG_PATHMOVE:
		xlnx_pcie_xvc_execute_pathmove(cmd);
		break;
	case JTAG_SCAN:
		return xlnx_pcie_xvc_execute_scan(cmd);
	case JTAG_RESET:
		xlnx_pcie_xvc_execute_reset(cmd);
		break;
	case JTAG_SLEEP:
		xlnx_pcie_xvc_execute_sleep(cmd);
		break;
	case JTAG_TMS:
		return xlnx_pcie_xvc_execute_tms(cmd);
	default:
		LOG_ERROR("BUG: Unknown JTAG command type encountered.");
		return ERROR_JTAG_QUEUE_FAILED;
	}

	return ERROR_OK;
}

static int xlnx_pcie_xvc_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int ret;

	while (cmd) {
		ret = xlnx_pcie_xvc_execute_command(cmd);

		if (ret != ERROR_OK)
			return ret;

		cmd = cmd->next;
	}

	return ERROR_OK;
}


static int xlnx_pcie_xvc_init(void)
{
	char filename[PATH_MAX];
	uint32_t cap, vh;

	snprintf(filename, PATH_MAX, "/sys/bus/pci/devices/%s/config",
		 xlnx_pcie_xvc->device);
	xlnx_pcie_xvc->fd = open(filename, O_RDWR | O_SYNC);
	if (xlnx_pcie_xvc->fd < 0) {
		LOG_ERROR("Failed to open device: %s", filename);
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_INFO("Scanning PCIe device %s's for Xilinx XVC/PCIe ...",
		 xlnx_pcie_xvc->device);
	/* Parse the PCIe extended capability list and try to find
	 * vendor specific header */
	xlnx_pcie_xvc->offset = PCIE_EXT_CAP_LST;
	while (xlnx_pcie_xvc->offset <= PCI_CFG_SPACE_EXP_SIZE - 4 &&
	       xlnx_pcie_xvc->offset >= PCIE_EXT_CAP_LST) {
		cap = xlnx_pcie_xvc_read_reg(XLNX_XVC_EXT_CAP);
		LOG_DEBUG("Checking capability at 0x%x; id=0x%04x version=0x%x next=0x%x",
			 xlnx_pcie_xvc->offset,
			 PCI_EXT_CAP_ID(cap),
			 PCI_EXT_CAP_VER(cap),
			 PCI_EXT_CAP_NEXT(cap));
		if (PCI_EXT_CAP_ID(cap) == PCI_EXT_CAP_ID_VNDR) {
			vh = xlnx_pcie_xvc_read_reg(XLNX_XVC_VSEC_HDR);
			LOG_DEBUG("Checking possible match at 0x%x; id: 0x%x; rev: 0x%x; length: 0x%x",
				 xlnx_pcie_xvc->offset,
				 PCI_VNDR_HEADER_ID(vh),
				 PCI_VNDR_HEADER_REV(vh),
				 PCI_VNDR_HEADER_LEN(vh));
			if ((PCI_VNDR_HEADER_ID(vh) == XLNX_XVC_VSEC_ID) &&
			    (PCI_VNDR_HEADER_LEN(vh) == XLNX_XVC_CAP_SIZE))
				break;
		}
		xlnx_pcie_xvc->offset = PCI_EXT_CAP_NEXT(cap);
	}
	if ((xlnx_pcie_xvc->offset > PCI_CFG_SPACE_EXP_SIZE - XLNX_XVC_CAP_SIZE) ||
	     xlnx_pcie_xvc->offset < PCIE_EXT_CAP_LST) {
		close(xlnx_pcie_xvc->fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_INFO("Found Xilinx XVC/PCIe capability at offset: 0x%x", xlnx_pcie_xvc->offset);

	return ERROR_OK;
}

static int xlnx_pcie_xvc_quit(void)
{
	int err;

	err = close(xlnx_pcie_xvc->fd);
	if (err)
		return err;

	return ERROR_OK;
}

COMMAND_HANDLER(xlnx_pcie_xvc_handle_config_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* we can't really free this in a safe manner, so at least
	 * limit the memory we're leaking by freeing the old one first
	 * before allocating a new one ...
	 */
	if (xlnx_pcie_xvc->device)
		free(xlnx_pcie_xvc->device);

	xlnx_pcie_xvc->device = strdup(CMD_ARGV[0]);
	return ERROR_OK;
}

static const struct command_registration xlnx_pcie_xvc_command_handlers[] = {
	{
		.name = "xlnx_pcie_xvc_config",
		.handler = xlnx_pcie_xvc_handle_config_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure XVC/PCIe JTAG adapter",
		.usage = "device",
	},
	COMMAND_REGISTRATION_DONE
};

static const char * const xlnx_pcie_xvc_transports[] = { "jtag", NULL };

struct jtag_interface xlnx_pcie_xvc_interface = {
	.name = "xlnx_pcie_xvc",
	.commands = xlnx_pcie_xvc_command_handlers,
	.transports = xlnx_pcie_xvc_transports,
	.execute_queue = &xlnx_pcie_xvc_execute_queue,
	.speed = NULL,
	.speed_div = NULL,
	.khz = NULL,
	.init = &xlnx_pcie_xvc_init,
	.quit = &xlnx_pcie_xvc_quit,
};
