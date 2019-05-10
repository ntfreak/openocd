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

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/types.h>
#include <helper/system.h>
#include <helper/time_support.h>
#include <helper/list.h>

#define RSH_MMIO_CHANNEL_RSHIM	0x1
#define RSHIM_CS_ROM_BASE	0x10000000

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

/* Use local variable stub for DP/AP registers. */
static uint32_t dp_ctrl_stat;
static uint32_t dp_id_code;
static uint32_t ap_sel, ap_bank;
static uint32_t ap_csw;
static uint32_t ap_drw;
static uint32_t ap_tar, ap_tar_inc;

/* ROM base address of up to 8 tiles (clusters). */
#define RSH_TILE_NUM	8
static uint32_t tile_rom_base[RSH_TILE_NUM + 1];

/* Static functions to read/write via rshim/coresight. */
static int (*rshim_read_rshim)(int chan, int addr, uint64_t *value);
static int (*rshim_write_rshim)(int chan, int addr, uint64_t value);
static int coresight_write(uint32_t tile, uint32_t addr, uint32_t wdata);
static int coresight_read(uint32_t tile, uint32_t addr, uint32_t *value);

/* RShim file handler. */
static int rshim_fd = -1;

static int rshim_dev_read_rshim(int chan, int addr, uint64_t *value)
{
	addr = (addr & 0xFFFF) | (1 << 16);
	return pread(rshim_fd, value, sizeof(*value), addr);
}

static int rshim_dev_write_rshim(int chan, int addr, uint64_t value)
{
	addr = (addr & 0xFFFF) | (1 << 16);
	return pwrite(rshim_fd, &value, sizeof(value), addr);
}

/*
 * Return index "i" corresponding to tile_rom_base[i - 1], or else 0 is
 * returned which indicates the root CS_ROM.
 */
static int addr_to_tile(uint32_t addr)
{
	int i;

	for (i = 0; i < RSH_TILE_NUM; i++) {
		if (!tile_rom_base[i])
			continue;
		if (addr < tile_rom_base[i])
			return i;
		if (i < RSH_TILE_NUM - 1 && tile_rom_base[i + 1] != 0 &&
		    addr >= tile_rom_base[i + 1])
			continue;
		return i + 1;
	}

	return ERROR_OK;
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

	if (!rshim_read_rshim || !rshim_write_rshim)
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

	rshim_write_rshim(RSH_MMIO_CHANNEL_RSHIM, RSH_CORESIGHT_CTL, ctl);

	do {
		rc = rshim_read_rshim(RSH_MMIO_CHANNEL_RSHIM,
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

	if (!rshim_read_rshim || !rshim_write_rshim)
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

	rshim_write_rshim(RSH_MMIO_CHANNEL_RSHIM, RSH_CORESIGHT_CTL, ctl);

	do {
		rc = rshim_read_rshim(RSH_MMIO_CHANNEL_RSHIM,
			RSH_CORESIGHT_CTL, &ctl);
		if (rc < 0) {
			LOG_ERROR("Failed to write rshim.\n");
			return rc;
		}
	} while (RSH_CS_GET_FIELD(ctl, GO));

	*value = RSH_CS_GET_FIELD(ctl, DATA);
	return 0;
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
		*data = RSHIM_CS_ROM_BASE;
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
		addr -= RSHIM_CS_ROM_BASE;
		tile = addr_to_tile(addr);
		if (tile > 0)
			addr -= tile_rom_base[tile - 1];
		rc = coresight_read(tile, addr, data);
		break;

	case MEM_AP_REG_DRW:
		addr = (ap_tar & ~0x3) + ap_tar_inc;
		addr -= RSHIM_CS_ROM_BASE;
		tile = addr_to_tile(addr);
		if (tile > 0)
			addr -= tile_rom_base[tile - 1];
		rc = coresight_read(tile, addr, data);
		if (!rc && (ap_csw & CSW_ADDRINC_MASK))
			ap_tar_inc += (ap_csw & 0x03) * 2;
		break;

	default:
		LOG_INFO("Unknown command");
		rc = ERROR_FAIL;
		break;
	}

	return rc;
}

static int rshim_ap_q_write(struct adiv5_ap *ap, unsigned int reg,
			    uint32_t data)
{
	int rc, tile;
	uint32_t addr;

	if (ap_bank != 0)
		return -EINVAL;

	switch (reg) {
	case MEM_AP_REG_CSW:
		ap_csw = data;
		return ERROR_OK;

	case MEM_AP_REG_TAR:
		ap_tar = data;
		ap_tar_inc = 0;
		return ERROR_OK;

	case MEM_AP_REG_BD0:
	case MEM_AP_REG_BD1:
	case MEM_AP_REG_BD2:
	case MEM_AP_REG_BD3:
		addr = (ap_tar & ~0xf) + (reg & 0x0C);
		addr -= RSHIM_CS_ROM_BASE;
		tile = addr_to_tile(addr);
		if (tile > 0)
			addr -= tile_rom_base[tile - 1];
		return coresight_write(tile, addr, data);

	case MEM_AP_REG_DRW:
		ap_drw = data;
		addr = (ap_tar & ~0x3) + ap_tar_inc;
		addr -= RSHIM_CS_ROM_BASE;
		tile = addr_to_tile(addr);
		if (tile > 0)
			addr -= tile_rom_base[tile - 1];
		rc = coresight_write(tile, addr, data);
		if (!rc && (ap_csw & CSW_ADDRINC_MASK))
			ap_tar_inc += (ap_csw & 0x03) * 2;
		return rc;

	default:
		return -EINVAL;
	}
}

static int rshim_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	return ERROR_OK;
}

static int rshim_dp_run(struct adiv5_dap *dap)
{
	return ERROR_OK;
}

static int rshim_dp_sync(struct adiv5_dap *dap)
{
	return ERROR_OK;
}

int rshim_connect(struct adiv5_dap *dap)
{
	uint32_t addr, entry, tile = 0;
	uint64_t data;
	char *dest;

	/* Specify default device path if RSHIM_DEST is not set. */
	dest = getenv("RSHIM_DEST");
	if (!dest)
		dest = "/dev/rshim0/rshim";

	rshim_fd = open(dest, O_RDWR | O_SYNC);
	if (rshim_fd == -1) {
		LOG_ERROR("Unable to open %s(%m)\n", dest);
		return -ENODEV;
	}

	/*
	 * Set read/write operation via the device file. Funtion pointers
	 * are used here so more ways like remote accessing via socket could
	 * be added later.
	 */
	rshim_read_rshim = rshim_dev_read_rshim;
	rshim_write_rshim = rshim_dev_write_rshim;

	/* Read the ROM table to get the address for the tiles. */
	for (addr = 0; addr < 0x1000; addr += 4) {
		data = coresight_read(0, addr, &entry);
		if (data || !entry)
			break;
		if (!(entry & 1))
			continue;
		tile_rom_base[tile++] = entry & 0xFFFFF000;
		if (tile >= RSH_TILE_NUM) {
			LOG_WARNING("Found more tiles than expected.");
			break;
		}
	}

	return ERROR_OK;
}

/* DAP operations. */
const struct dap_ops rshim_dap_ops = {
	.connect             = rshim_connect,
	.queue_dp_read       = rshim_dp_q_read,
	.queue_dp_write      = rshim_dp_q_write,
	.queue_ap_read       = rshim_ap_q_read,
	.queue_ap_write      = rshim_ap_q_write,
	.queue_ap_abort      = rshim_ap_q_abort,
	.run                 = rshim_dp_run,
	.sync                = rshim_dp_sync,
};
