/*
 * Copyright (c) 2020, Mellanox Technologies Ltd. - All Rights Reserved
 * Liming Sun <lsun@mellanox.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/types.h>
#include <helper/system.h>
#include <helper/time_support.h>
#include <helper/list.h>
#include <jtag/interface.h>
#ifdef HAVE_SYS_IOCTL_H
#include <sys/ioctl.h>
#endif
#include <target/arm_adi_v5.h>
#include <transport/transport.h>

/* Rshim channel where the CoreSight register resides. */
#define RSH_MMIO_CHANNEL_RSHIM	0x1

/* APB and tile address translation. */
#define RSH_CS_ROM_BASE		0x80000000
#define RSH_CS_TILE_BASE	0x44000000
#define RSH_CS_TILE_SIZE	0x04000000

/*
 * APB-AP Identification Register
 * The default value is defined in "CoreSight on-chip trace and debug
 * (Revision: r1p0)", Section 3.16.5 APB-AP register summary.
 */
#define APB_AP_IDR			0x44770002

/* CoreSight register definition. */
#define RSH_CORESIGHT_CTL		0x0e00
#define RSH_CORESIGHT_CTL_GO_SHIFT	0
#define RSH_CORESIGHT_CTL_GO_MASK	0x1ULL
#define RSH_CORESIGHT_CTL_ACTION_SHIFT	1
#define RSH_CORESIGHT_CTL_ACTION_MASK	0x2ULL
#define RSH_CORESIGHT_CTL_ADDR_SHIFT	2
#define RSH_CORESIGHT_CTL_ADDR_MASK	0x7ffffffcULL
#define RSH_CORESIGHT_CTL_ERR_SHIFT	31
#define RSH_CORESIGHT_CTL_ERR_MASK	0x80000000ULL
#define RSH_CORESIGHT_CTL_DATA_SHIFT	32
#define RSH_CORESIGHT_CTL_DATA_MASK	0xffffffff00000000ULL

/* Util macros to access the CoreSight register. */
#define RSH_CS_GET_FIELD(reg, field) \
	(((uint64_t)(reg) & RSH_CORESIGHT_CTL_##field##_MASK) >> \
		RSH_CORESIGHT_CTL_##field##_SHIFT)

#define RSH_CS_SET_FIELD(reg, field, value) \
	(reg) = (((reg) & ~RSH_CORESIGHT_CTL_##field##_MASK) | \
		(((uint64_t)(value) << RSH_CORESIGHT_CTL_##field##_SHIFT) & \
		RSH_CORESIGHT_CTL_##field##_MASK))

#ifdef HAVE_SYS_IOCTL_H
/* Message used to program rshim via ioctl(). */
typedef struct {
	uint32_t addr;
	uint64_t data;
} __attribute__((packed)) rshim_ioctl_msg;

enum {
	RSH_IOC_READ = _IOWR('R', 0, rshim_ioctl_msg),
	RSH_IOC_WRITE = _IOWR('R', 1, rshim_ioctl_msg),
};
#endif

/* Use local variable stub for DP/AP registers. */
static uint32_t dp_ctrl_stat;
static uint32_t dp_id_code;
static uint32_t ap_sel, ap_bank;
static uint32_t ap_csw;
static uint32_t ap_drw;
static uint32_t ap_tar, ap_tar_inc;

/* Static functions to read/write via rshim/coresight. */
static int (*rshim_read)(int chan, int addr, uint64_t *value);
static int (*rshim_write)(int chan, int addr, uint64_t value);
static int coresight_write(uint32_t tile, uint32_t addr, uint32_t wdata);
static int coresight_read(uint32_t tile, uint32_t addr, uint32_t *value);

/* RShim file handler. */
static int rshim_fd = -1;

/* DAP error code. */
static int rshim_dap_retval = ERROR_OK;

/* Default rshim device. */
#define RSHIM_DEV_PATH_DEFAULT	"/dev/rshim0/rshim"
static char *rshim_dev_path;

static int rshim_dev_read(int chan, int addr, uint64_t *value)
{
	int rc;

	addr = (addr & 0xFFFF) | (1 << 16);
	rc = pread(rshim_fd, value, sizeof(*value), addr);

#ifdef HAVE_SYS_IOCTL_H
	if (rc < 0 && errno == ENOSYS) {
		rshim_ioctl_msg msg;

		msg.addr = addr;
		msg.data = 0;
		rc = ioctl(rshim_fd, RSH_IOC_READ, &msg);
		if (!rc)
			*value = msg.data;
	}
#endif

	return rc;
}

static int rshim_dev_write(int chan, int addr, uint64_t value)
{
	int rc;

	addr = (addr & 0xFFFF) | (1 << 16);
	rc = pwrite(rshim_fd, &value, sizeof(value), addr);

#ifdef HAVE_SYS_IOCTL_H
	if (rc < 0 && errno == ENOSYS) {
		rshim_ioctl_msg msg;

		msg.addr = addr;
		msg.data = value;
		rc = ioctl(rshim_fd, RSH_IOC_WRITE, &msg);
	}
#endif

	return rc;
}

/* Convert AP address to tile local address. */
static void ap_addr_2_tile(int *tile, uint32_t *addr)
{
	*addr -= RSH_CS_ROM_BASE;

	if (*addr < RSH_CS_TILE_BASE) {
		*tile = 0;
	} else {
		*addr -= RSH_CS_TILE_BASE;
		*tile = *addr / RSH_CS_TILE_SIZE + 1;
		*addr = *addr % RSH_CS_TILE_SIZE;
	}
}

/*
 * Write 4 bytes on the APB bus.
 * tile = 0: access the root CS_ROM table
 *      > 0: access the ROM table of cluster (tile - 1)
 */
static int coresight_write(uint32_t tile, uint32_t addr, uint32_t wdata)
{
	uint64_t ctl = 0;
	int rc;

	if (!rshim_read || !rshim_write)
		return ERROR_FAIL;

	/*
	 * ADDR[28]    - must be set to 1 due to coresight ip.
	 * ADDR[27:24] - linear tile id
	 */
	addr = (addr >> 2) | (tile << 24);
	if (tile)
		addr |= (1 << 28);
	RSH_CS_SET_FIELD(ctl, ADDR, addr);
	RSH_CS_SET_FIELD(ctl, ACTION, 0);	/* write */
	RSH_CS_SET_FIELD(ctl, DATA, wdata);
	RSH_CS_SET_FIELD(ctl, GO, 1);		/* start */

	rshim_write(RSH_MMIO_CHANNEL_RSHIM, RSH_CORESIGHT_CTL, ctl);

	do {
		rc = rshim_read(RSH_MMIO_CHANNEL_RSHIM,
				RSH_CORESIGHT_CTL, &ctl);
		if (rc < 0) {
			LOG_ERROR("Failed to read rshim.\n");
			return rc;
		}
	} while (RSH_CS_GET_FIELD(ctl, GO));

	return ERROR_OK;
}

static int coresight_read(uint32_t tile, uint32_t addr, uint32_t *value)
{
	uint64_t ctl = 0;
	int rc;

	if (!rshim_read || !rshim_write)
		return ERROR_FAIL;

	/*
	 * ADDR[28]    - must be set to 1 due to coresight ip.
	 * ADDR[27:24] - linear tile id
	 */
	addr = (addr >> 2) | (tile << 24);
	if (tile)
		addr |= (1 << 28);
	RSH_CS_SET_FIELD(ctl, ADDR, addr);
	RSH_CS_SET_FIELD(ctl, ACTION, 1);	/* read */
	RSH_CS_SET_FIELD(ctl, GO, 1);		/* start */

	rshim_write(RSH_MMIO_CHANNEL_RSHIM, RSH_CORESIGHT_CTL, ctl);

	do {
		rc = rshim_read(RSH_MMIO_CHANNEL_RSHIM,
				RSH_CORESIGHT_CTL, &ctl);
		if (rc < 0) {
			LOG_ERROR("Failed to write rshim.\n");
			return rc;
		}
	} while (RSH_CS_GET_FIELD(ctl, GO));

	*value = RSH_CS_GET_FIELD(ctl, DATA);
	return ERROR_OK;
}

static int rshim_dp_q_read(struct adiv5_dap *dap, unsigned int reg,
			   uint32_t *data)
{
	if (!data)
		return ERROR_OK;

	switch (reg) {
	case DP_DPIDR:
		*data = dp_id_code;
		break;

	case DP_CTRL_STAT:
		*data = CDBGPWRUPACK | CSYSPWRUPACK;
		break;

	default:
		break;
	}

	return ERROR_OK;
}

static int rshim_dp_q_write(struct adiv5_dap *dap, unsigned int reg,
			    uint32_t data)
{
	switch (reg) {
	case DP_CTRL_STAT:
		dp_ctrl_stat = data;
		break;
	case DP_SELECT:
		ap_sel = (data & DP_SELECT_APSEL) >> 24;
		ap_bank = (data & DP_SELECT_APBANK) >> 4;
		break;
	default:
		LOG_INFO("Unknown command");
		break;
	}

	return ERROR_OK;
}

static int rshim_ap_q_read(struct adiv5_ap *ap, unsigned int reg,
			   uint32_t *data)
{
	uint32_t addr;
	int rc = ERROR_OK, tile;

	switch (reg) {
	case MEM_AP_REG_CSW:
		*data = ap_csw;
		break;

	case MEM_AP_REG_CFG:
		*data = 0;
		break;

	case MEM_AP_REG_BASE:
		*data = RSH_CS_ROM_BASE;
		break;

	case AP_REG_IDR:
		if (ap->ap_num == 0)
			*data = APB_AP_IDR;
		else
			*data = 0;
		break;

	case MEM_AP_REG_BD0:
	case MEM_AP_REG_BD1:
	case MEM_AP_REG_BD2:
	case MEM_AP_REG_BD3:
		addr = (ap_tar & ~0xf) + (reg & 0x0C);
		ap_addr_2_tile(&tile, &addr);
		rc = coresight_read(tile, addr, data);
		break;

	case MEM_AP_REG_DRW:
		addr = (ap_tar & ~0x3) + ap_tar_inc;
		ap_addr_2_tile(&tile, &addr);
		rc = coresight_read(tile, addr, data);
		if (!rc && (ap_csw & CSW_ADDRINC_MASK))
			ap_tar_inc += (ap_csw & 0x03) * 2;
		break;

	default:
		LOG_INFO("Unknown command");
		rc = ERROR_FAIL;
		break;
	}

	/* Track the last error code. */
	if (rc != ERROR_OK)
		rshim_dap_retval = rc;

	return rc;
}

static int rshim_ap_q_write(struct adiv5_ap *ap, unsigned int reg,
			    uint32_t data)
{
	int rc = ERROR_OK, tile;
	uint32_t addr;

	if (ap_bank != 0) {
		rshim_dap_retval = ERROR_FAIL;
		return ERROR_FAIL;
	}

	switch (reg) {
	case MEM_AP_REG_CSW:
		ap_csw = data;
		break;

	case MEM_AP_REG_TAR:
		ap_tar = data;
		ap_tar_inc = 0;
		break;

	case MEM_AP_REG_BD0:
	case MEM_AP_REG_BD1:
	case MEM_AP_REG_BD2:
	case MEM_AP_REG_BD3:
		addr = (ap_tar & ~0xf) + (reg & 0x0C);
		ap_addr_2_tile(&tile, &addr);
		rc = coresight_write(tile, addr, data);
		break;

	case MEM_AP_REG_DRW:
		ap_drw = data;
		addr = (ap_tar & ~0x3) + ap_tar_inc;
		ap_addr_2_tile(&tile, &addr);
		rc = coresight_write(tile, addr, data);
		if (!rc && (ap_csw & CSW_ADDRINC_MASK))
			ap_tar_inc += (ap_csw & 0x03) * 2;
		break;

	default:
		rc = EINVAL;
		break;
	}

	/* Track the last error code. */
	if (rc != ERROR_OK)
		rshim_dap_retval = rc;

	return rc;
}

static int rshim_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	return ERROR_OK;
}

static int rshim_dp_run(struct adiv5_dap *dap)
{
	int retval = rshim_dap_retval;

	/* Clear the error code. */
	rshim_dap_retval = ERROR_OK;

	return retval;
}

static int rshim_connect(struct adiv5_dap *dap)
{
	char *path = rshim_dev_path ? rshim_dev_path : RSHIM_DEV_PATH_DEFAULT;

	rshim_fd = open(path, O_RDWR | O_SYNC);
	if (rshim_fd == -1) {
		LOG_ERROR("Unable to open %s\n", path);
		return ERROR_FAIL;
	}

	/*
	 * Set read/write operation via the device file. Function pointers
	 * are used here so more ways like remote accessing via socket could
	 * be added later.
	 */
	rshim_read = rshim_dev_read;
	rshim_write = rshim_dev_write;

	return ERROR_OK;
}

static void rshim_disconnect(struct adiv5_dap *dap)
{
	if (rshim_fd != -1) {
		close(rshim_fd);
		rshim_fd = -1;
	}
}

COMMAND_HANDLER(rshim_dap_device_command)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Too many arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	free(rshim_dev_path);
	rshim_dev_path = strdup(CMD_ARGV[0]);
	return ERROR_OK;
}

static const struct command_registration rshim_dap_subcommand_handlers[] = {
	{
		.name = "device",
		.handler = rshim_dap_device_command,
		.mode = COMMAND_CONFIG,
		.help = "set the rshim device",
		.usage = "</dev/rshim<N>/rshim>",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration rshim_dap_command_handlers[] = {
	{
		.name = "rshim",
		.mode = COMMAND_ANY,
		.help = "perform rshim management",
		.chain = rshim_dap_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int rshim_dap_init(void)
{
	return ERROR_OK;
}

static int rshim_dap_quit(void)
{
	return ERROR_OK;
}

static int rshim_dap_reset(int req_trst, int req_srst)
{
	return ERROR_OK;
}

static int rshim_dap_speed(int speed)
{
	return ERROR_OK;
}

static int rshim_dap_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

static int rshim_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

/* DAP operations. */
static const struct dap_ops rshim_dap_ops = {
	.connect = rshim_connect,
	.queue_dp_read = rshim_dp_q_read,
	.queue_dp_write = rshim_dp_q_write,
	.queue_ap_read = rshim_ap_q_read,
	.queue_ap_write = rshim_ap_q_write,
	.queue_ap_abort = rshim_ap_q_abort,
	.run = rshim_dp_run,
	.quit = rshim_disconnect,
};

static const char *const rshim_dap_transport[] = { "dapdirect_swd", NULL };

struct adapter_driver rshim_dap_adapter_driver = {
	.name = "rshim",
	.transports = rshim_dap_transport,
	.commands = rshim_dap_command_handlers,

	.init = rshim_dap_init,
	.quit = rshim_dap_quit,
	.reset = rshim_dap_reset,
	.speed = rshim_dap_speed,
	.khz = rshim_dap_khz,
	.speed_div = rshim_dap_speed_div,

	.dap_swd_ops = &rshim_dap_ops,
};
