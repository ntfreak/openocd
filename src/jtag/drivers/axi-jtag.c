// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 Google LLC.
 * Author: Moritz Fischer <moritzf@google.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <jtag/interface.h>
#include <jtag/swd.h>
#include <jtag/commands.h>
#include <helper/bits.h>
#include <helper/replacements.h>
#include <sys/mman.h>

#define AXI_JTAG_LEN_REG	0x00
#define AXI_JTAG_TDI_REG	0x04
#define AXI_JTAG_TMS_REG	0x08
#define AXI_JTAG_TDO_REG	0x0c
#define AXI_JTAG_CTRL_REG	0x10
# define AXI_JTAG_CTRL_BUSY	BIT(0)
# define AXI_JTAG_CTRL_WRITE	BIT(0)
# define AXI_JTAG_CTRL_TRST	BIT(1)
# define AXI_JTAG_CTRL_SRST	BIT(2)
#define AXI_JTAG_PRSC_REG	0x14
# define AXI_JTAG_PRSC_MASK	0xff

#define AXI_JTAG_MAX_BITS	0x20
#define AXI_JTAG_MAP_SIZE	0x1000
#define AXI_JTAG_MAP_MASK	(AXI_JTAG_MAP_SIZE - 1)

#if defined(__arm__)
#define wmb() asm volatile("dmb st" : : : "memory")
#elif defined(__aarch64__)
#define wmb() asm volatile("dmb st" : : : "memory")
#elif defined(__powerpc__)
#define wmb() asm volatile("sync" : : : "memory")
#elif defined(__x86_64__) || defined(__i386__)
#define wmb() asm volatile("sfence" : : : "memory")
#elif defined(__riscv) || defined(__riscv__)
#define wmb() asm volatile("fence w,o" : : : "memory")
#else
#define wmb() do { } while (0)
#endif

struct axi_jtag {
	int fd;
	void *io_base;
	void *io_vaddr;
	off_t offset;

	int input_clk_khz;

	char *device_node;
};

static struct axi_jtag axi_jtag_state;
static struct axi_jtag *axi_jtag = &axi_jtag_state;

static uint32_t axi_jtag_read_reg(const off_t offset)
{
	return  *((volatile uint32_t*)(axi_jtag->io_vaddr + offset));
}

static void axi_jtag_write_reg(const off_t offset, const uint32_t val)
{
	/* This may be overly paranoid, but when going through width
	 * conversions strange things happen inside AXI IP sometimes
	 * Problems without barrier were observed on ZynqMP aarch64
	 */
	wmb();
	*((volatile uint32_t*)(axi_jtag->io_vaddr + offset)) = val;
}

static int axi_jtag_is_busy(void)
{
	return axi_jtag_read_reg(AXI_JTAG_CTRL_REG) & AXI_JTAG_CTRL_BUSY;
}

static void axi_jtag_transact(size_t num_bits, uint32_t tms, uint32_t tdi,
			      uint32_t *tdo)
{
	axi_jtag_write_reg(AXI_JTAG_LEN_REG, num_bits);
	axi_jtag_write_reg(AXI_JTAG_TDI_REG, tdi);
	axi_jtag_write_reg(AXI_JTAG_TMS_REG, tms);
	axi_jtag_write_reg(AXI_JTAG_CTRL_REG, AXI_JTAG_CTRL_WRITE);
	while (axi_jtag_is_busy())
		;
	if (tdo)
		*tdo = axi_jtag_read_reg(AXI_JTAG_TDO_REG);

	LOG_DEBUG_IO("Transact num_bits: %zu, tms: %x, tdi: %x, tdo: %x",
		     num_bits, tms, tdi, tdo ? *tdo : 0xdeadbeef);
}

void axi_jtag_execute_stableclocks(struct jtag_command *cmd)
{
	int tms = tap_get_state() == TAP_RESET ? 1 : 0;
	size_t left = cmd->cmd.stableclocks->num_cycles;
	size_t write;

	LOG_DEBUG_IO("stableclocks %i cycles", cmd->cmd.runtest->num_cycles);

	while (left) {
		write = MIN(AXI_JTAG_MAX_BITS, left);
		axi_jtag_transact(write, tms, 0, NULL);
		left -= write;
	};
}

static void axi_jtag_execute_statemove(size_t skip)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(),
					    tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(),
					     tap_get_end_state());

	LOG_DEBUG_IO("statemove starting at (skip: %zu) %s end in %s", skip,
		     tap_state_name(tap_get_state()),
		     tap_state_name(tap_get_end_state()));


	axi_jtag_transact(tms_count - skip, tms_scan >> skip, 0, NULL);
	tap_set_state(tap_get_end_state());
}

static void axi_jtag_execute_runtest(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("runtest %i cycles, end in %i",
		     cmd->cmd.runtest->num_cycles,
		     cmd->cmd.runtest->end_state);

	tap_state_t tmp_state = tap_get_end_state();

	if (tap_get_state() != TAP_IDLE) {
		tap_set_end_state(TAP_IDLE);
		axi_jtag_execute_statemove(0);
	};

	size_t left = cmd->cmd.runtest->num_cycles;
	size_t write;

	while (left) {
		write = MIN(AXI_JTAG_MAX_BITS, left);
		axi_jtag_transact(write, 0, 0, NULL);
		left -= write;
	};

	tap_set_end_state(tmp_state);
	if (tap_get_state() != tap_get_end_state())
		axi_jtag_execute_statemove(0);
}

static void axi_jtag_execute_pathmove(struct jtag_command *cmd)
{
	size_t num_states = cmd->cmd.pathmove->num_states;
	tap_state_t *path = cmd->cmd.pathmove->path;
	size_t i;

	LOG_DEBUG_IO("pathmove: %i states, end in %i",
		cmd->cmd.pathmove->num_states,
		cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	for (i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false))
			axi_jtag_transact(1, 1, 0, NULL);
		else if (path[i] == tap_state_transition(tap_get_state(), true))
			axi_jtag_transact(1, 0, 0, NULL);
		else
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				  tap_state_name(tap_get_state()),
				  tap_state_name(path[i]));
		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static int axi_jtag_execute_scan(struct jtag_command *cmd)
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
	LOG_DEBUG_IO("%s scan type %d %d bits; starts in %s end in %s",
		     (cmd->cmd.scan->ir_scan) ? "IR" : "DR", type, scan_size,
		     tap_state_name(tap_get_state()),
		     tap_state_name(cmd->cmd.scan->end_state));

	/* If we're in TAP_DR_SHIFT state but need to do a IR_SCAN or
	 * vice-versa, do a statemove to corresponding other state, then restore
	 * end state
	 */
	if (ir_scan && tap_get_state() != TAP_IRSHIFT) {
		tap_set_end_state(TAP_IRSHIFT);
		axi_jtag_execute_statemove(0);
		tap_set_end_state(saved_end_state);
	} else if (!ir_scan && (tap_get_state() != TAP_DRSHIFT)) {
		tap_set_end_state(TAP_DRSHIFT);
		axi_jtag_execute_statemove(0);
		tap_set_end_state(saved_end_state);
	}

	left = scan_size;
	while (left) {
		write = MIN(AXI_JTAG_MAX_BITS, left);
		/* the last TMS should be a 1, to leave the state */
		tms = left <= AXI_JTAG_MAX_BITS ? BIT(write - 1) : 0;
		tdi = (type != SCAN_IN) ? buf_get_u32(rd_ptr, 0, write) : 0;
		axi_jtag_transact(write, tms, tdi, type != SCAN_OUT ?
				  &tdo : NULL);
		left -= write;
		buf_set_u32(rd_ptr, 0, write, tdo);
		rd_ptr += sizeof(uint32_t);
	};

	err = jtag_read_buffer(buf, cmd->cmd.scan);
	if (buf)
		free(buf);

	if (tap_get_state() != tap_get_end_state())
		axi_jtag_execute_statemove(1);

	return err;
}

static void axi_jtag_execute_reset(struct jtag_command *cmd)
{
	uint32_t ctrl;

	ctrl = axi_jtag_read_reg(AXI_JTAG_CTRL_REG);
	ctrl &= ~(AXI_JTAG_CTRL_TRST | AXI_JTAG_CTRL_SRST);

	if (cmd->cmd.reset->trst)
		ctrl |= AXI_JTAG_CTRL_TRST;
	if (cmd->cmd.reset->srst)
		ctrl |= AXI_JTAG_CTRL_SRST;
	axi_jtag_write_reg(AXI_JTAG_CTRL_REG, ctrl);

	/* If trst is set, reset the TAP, same goes for special
	 * configurations such as RESET_SRST_PULLS_TRST
	 */
	if ((cmd->cmd.reset->trst == 1) ||
	    (cmd->cmd.reset->srst &&
	    (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
		tap_set_state(TAP_RESET);

	LOG_DEBUG_IO("reset trst: %i srst: %i", cmd->cmd.reset->trst,
		     cmd->cmd.reset->srst);
}

static void axi_jtag_execute_sleep(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("sleep %" PRIi32 "", cmd->cmd.sleep->us);
	usleep(cmd->cmd.sleep->us);
}

static int axi_jtag_execute_tms(struct jtag_command *cmd)
{
	const size_t num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	size_t left, write;
	uint32_t tms;

	LOG_DEBUG_IO("execute tms %zu", num_bits);

	left = num_bits;
	while (left) {
		write = MIN(AXI_JTAG_MAX_BITS, left);
		tms = buf_get_u32(bits, 0, write);
		axi_jtag_transact(write, tms, 0, NULL);
		left -= write;
		bits += 4;
	};

	return ERROR_OK;
}

static int axi_jtag_execute_command(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("%s: cmd->type: %u", __func__, cmd->type);
	switch (cmd->type) {
	case JTAG_STABLECLOCKS:
		axi_jtag_execute_stableclocks(cmd);
		break;
	case JTAG_RUNTEST:
		axi_jtag_execute_runtest(cmd);
		break;
	case JTAG_TLR_RESET:
		tap_set_end_state(cmd->cmd.statemove->end_state);
		axi_jtag_execute_statemove(0);
		break;
	case JTAG_PATHMOVE:
		axi_jtag_execute_pathmove(cmd);
		break;
	case JTAG_SCAN:
		return axi_jtag_execute_scan(cmd);
	case JTAG_RESET:
		axi_jtag_execute_reset(cmd);
		break;
	case JTAG_SLEEP:
		axi_jtag_execute_sleep(cmd);
		break;
	case JTAG_TMS:
		return axi_jtag_execute_tms(cmd);
	default:
		LOG_ERROR("BUG: Unknown JTAG command type encountered.");
		return ERROR_JTAG_QUEUE_FAILED;
	}

	return ERROR_OK;
}

static int axi_jtag_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int ret;

	while (cmd) {
		ret = axi_jtag_execute_command(cmd);

		if (ret != ERROR_OK)
			return ret;

		cmd = cmd->next;
	}

	return ERROR_OK;
}

static int axi_jtag_init(void)
{
	axi_jtag->fd = open(axi_jtag->device_node, O_RDWR | O_SYNC);
	if (axi_jtag->fd < 0) {
		LOG_ERROR("Failed to open device: %s", axi_jtag->device_node);
		return ERROR_JTAG_INIT_FAILED;
	}

	axi_jtag->io_base = mmap(NULL, AXI_JTAG_MAP_SIZE,
				 PROT_READ | PROT_WRITE, MAP_SHARED,
				 axi_jtag->fd,
				 axi_jtag->offset & ~AXI_JTAG_MAP_MASK);
	if (axi_jtag->io_base == MAP_FAILED) {
		LOG_ERROR("Failed to mmap device: %s, offset %lx",
			  axi_jtag->device_node, axi_jtag->offset);
		goto err_mmap;
	}

	axi_jtag->io_vaddr = axi_jtag->io_base +
			     (axi_jtag->offset & AXI_JTAG_MAP_MASK);

	return ERROR_OK;

err_mmap:
	close(axi_jtag->fd);
	return ERROR_JTAG_INIT_FAILED;
}

static int axi_jtag_quit(void)
{
	int err;

	err = munmap(axi_jtag->io_base, getpagesize());
	if (err)
		goto err_unmap;

	err = close(axi_jtag->fd);
	if (err)
		return err;
	return ERROR_OK;

err_unmap:
	return ERROR_JTAG_QUEUE_FAILED;
}

static int axi_jtag_speed(int speed)
{
	int psc_val;

	if (!speed) {
		LOG_INFO("RTCK not supported");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	if (speed > axi_jtag->input_clk_khz) {
		LOG_INFO("Reduced speed request from %d kHz to %d kHz max",
			 speed, axi_jtag->input_clk_khz / 2);
		speed = axi_jtag->input_clk_khz / 2;
	}

	if (speed < (axi_jtag->input_clk_khz / AXI_JTAG_PRSC_MASK / 2)) {
		LOG_INFO("Increased speed request from %d kHz to %d kHz min",
			 speed, axi_jtag->input_clk_khz /
			 AXI_JTAG_PRSC_MASK / 2);
		speed = axi_jtag->input_clk_khz / AXI_JTAG_PRSC_MASK / 2;
	}

	/* Need to stay under the max given by speed, so round up the psc,
	 * but register value need to calculated value *minus* one
	 */
	psc_val = DIV_ROUND_UP(axi_jtag->input_clk_khz, (2 * speed)) - 1;
	axi_jtag_write_reg(AXI_JTAG_PRSC_REG, psc_val & AXI_JTAG_PRSC_MASK);

	return ERROR_OK;
}

static int axi_jtag_speed_div(int speed, int *khz)
{
	int psc;

	psc = DIV_ROUND_UP(axi_jtag->input_clk_khz, (2 * speed));
	*khz = axi_jtag->input_clk_khz / psc / 2;
	return ERROR_OK;
}

static int axi_jtag_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RTCK not supported");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	*jtag_speed = khz;
	return ERROR_OK;
}

COMMAND_HANDLER(axi_jtag_handle_config_command)
{
	switch (CMD_ARGC) {
	case 3:
		axi_jtag->input_clk_khz = strtol(CMD_ARGV[2], NULL, 0) / 1000;
		axi_jtag->offset = strtol(CMD_ARGV[1], NULL, 0);
		axi_jtag->device_node = strdup(CMD_ARGV[0]);
		break;
	case 2:
		axi_jtag->input_clk_khz = 100000;
		axi_jtag->offset = strtol(CMD_ARGV[1], NULL, 0);
		axi_jtag->device_node = strdup(CMD_ARGV[0]);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	};
	return ERROR_OK;
}

static const struct command_registration axi_jtag_command_handlers[] = {
	{
		.name = "axi_jtag_config",
		.handler = axi_jtag_handle_config_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure AXI JTAG device and offset",
		.usage = "device offset",
	},
	COMMAND_REGISTRATION_DONE
};

static const char * const axi_jtag_transports[] = { "jtag", NULL };

struct jtag_interface axi_jtag_interface = {
	.name = "axijtag",
	.commands = axi_jtag_command_handlers,
	.transports = axi_jtag_transports,
	.execute_queue = &axi_jtag_execute_queue,
	.speed = &axi_jtag_speed,
	.speed_div = &axi_jtag_speed_div,
	.khz = &axi_jtag_khz,
	.init = &axi_jtag_init,
	.quit = &axi_jtag_quit,
};
