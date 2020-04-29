/***************************************************************************
 *   Copyright (C) 2016-2017 by Nuvoton                                    *
 *   Zale Yu <cyyu@nuvoton.com>                                            *
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

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <target/target.h>

#include <target/cortex_m.h>

#include "libusb_helper.h"

#define ENDPOINT_IN  0x80
#define ENDPOINT_OUT 0x00

#define NULINK_WRITE_TIMEOUT 1000
#define NULINK_READ_TIMEOUT  1000

#define NULINK_RX_EP          (1|ENDPOINT_IN)
#define NULINK_TX_EP          (2|ENDPOINT_OUT)

#define NULINK_HID_MAX_SIZE   (64)
#define V6M_MAX_COMMAND_LENGTH (NULINK_HID_MAX_SIZE - 2)
#define V7M_MAX_COMMAND_LENGTH (NULINK_HID_MAX_SIZE - 2)

struct nulink_usb_handle_s {
	struct libusb_device_handle *fd;
	struct libusb_transfer *trans;
	uint8_t rx_ep;
	uint8_t tx_ep;
	uint32_t usbcmdidx;
	uint8_t cmdidx;
	uint8_t cmdbuf[NULINK_HID_MAX_SIZE];
	uint8_t tempbuf[NULINK_HID_MAX_SIZE];
	uint8_t databuf[NULINK_HID_MAX_SIZE];
	uint32_t max_mem_packet;
	enum hl_transports transport;
	uint16_t hardwareConfig; /* bit 0: 1:Nu-Link-Pro, 0:Nu-Link */
};

/* ICE Command */
#define CMD_READ_REG				0xB5UL
#define CMD_READ_RAM				0xB1UL
#define CMD_WRITE_REG				0xB8UL
#define CMD_WRITE_RAM				0xB9UL
#define CMD_CHECK_ID				0xA3UL
#define CMD_MCU_RESET				0xE2UL
#define CMD_CHECK_MCU_STOP			0xD8UL
#define CMD_MCU_STEP_RUN			0xD1UL
#define CMD_MCU_STOP_RUN			0xD2UL
#define CMD_MCU_FREE_RUN			0xD3UL
#define CMD_SET_CONFIG				0xA2UL

#define ARM_SRAM_BASE				0x20000000

#define HARDWARECONFIG_NULINKPRO    1

enum reset_e {
	RESET_AUTO = 0,
	RESET_HW = 1,
	RESET_SYSRESETREQ = 2,
	RESET_VECTRESET = 3,
	RESET_FAST_RESCUE = 4, /* Rescue and erase the chip, need very fast speed */
};

enum connect_e {
	CONNECT_NORMAL = 0,      /* Support all reset method */
	CONNECT_PRE_RESET = 1,   /* Support all reset method */
	CONNECT_UNDER_RESET = 2, /* Support all reset method */
	CONNECT_NONE = 3,        /* Support RESET_HW, (RESET_AUTO = RESET_HW) */
	CONNECT_DISCONNECT = 4,  /* Support RESET_NONE, (RESET_AUTO = RESET_NONE) */
	CONNECT_ICP_MODE = 5     /* Support NUC505 ICP mode*/
};

static int nulink_usb_xfer_rw(void *handle, int cmdsize, uint8_t *buf)
{
	struct nulink_usb_handle_s *h = handle;
	int transferred;

	assert(handle != NULL);

	jtag_libusb_interrupt_write(h->fd, h->tx_ep, (char *)h->cmdbuf, cmdsize,
			NULINK_WRITE_TIMEOUT, &transferred);

	jtag_libusb_interrupt_read(h->fd, h->rx_ep, (char *)buf,
			cmdsize, NULINK_READ_TIMEOUT, &transferred);

	return ERROR_OK;
}

static int nulink_usb_xfer(void *handle, uint8_t *buf, int size)
{
	int err, cmdsize;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	cmdsize = NULINK_HID_MAX_SIZE;
	err = nulink_usb_xfer_rw(h, cmdsize, h->tempbuf);

	memcpy(buf, h->tempbuf + 2, NULINK_HID_MAX_SIZE - 2);

	return err;
}

static void nulink_usb_init_buffer(void *handle, uint32_t size)
{
	struct nulink_usb_handle_s *h = handle;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, NULINK_HID_MAX_SIZE);
	memset(h->tempbuf, 0, NULINK_HID_MAX_SIZE);
	memset(h->databuf, 0, NULINK_HID_MAX_SIZE);

	h->cmdbuf[0] = ++h->usbcmdidx & 0x7F;
	h->cmdbuf[1] = size;
	h->cmdidx += 2;
}

static int nulink_usb_version(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_version");

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, V6M_MAX_COMMAND_LENGTH);

	memset(h->cmdbuf + h->cmdidx, 0xFF, V6M_MAX_COMMAND_LENGTH);
	h->cmdbuf[h->cmdidx + 4] = 0xA1; /* host_rev_num: 6561 */;
	h->cmdbuf[h->cmdidx + 5] = 0x19;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 5);

	if (res != ERROR_OK)
		return res;

	LOG_INFO("NULINK firmware_version(%d), product_id(0x%08x)",
		le_to_h_u32(h->databuf),
		le_to_h_u32(h->databuf + 4 * 1));

	bool isNulinkPro = ((le_to_h_u32(h->databuf + 4 * 2) & 1) ? true : false);

	if (isNulinkPro) {
		LOG_INFO("NULINK is Nu-Link-Pro, target_voltage_mv(%d), usb_voltage_mv(%d)",
			(int)(le_to_h_u32(h->databuf + 4 * 3)),
			(int)(le_to_h_u32(h->databuf + 4 * 3) >> 16));

		h->hardwareConfig = (h->hardwareConfig & ~(HARDWARECONFIG_NULINKPRO)) | HARDWARECONFIG_NULINKPRO;
	} else {
		LOG_INFO("NULINK is Nu-Link");
	}

	return ERROR_OK;
}

static int nulink_usb_idcode(void *handle, uint32_t *idcode)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_idcode");

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CHECK_ID);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 2);

	if (res != ERROR_OK)
		return res;

	*idcode = le_to_h_u32(h->databuf + 4 * 1);

	LOG_INFO("IDCODE: 0x%08"PRIX32, *idcode);

	return ERROR_OK;
}

static int nulink_usb_write_debug_reg(void *handle, uint32_t addr, uint32_t val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_write_debug_reg(0x%08x): 0x%08x",
		addr,
		val);

	nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = 0x00;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = 0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, val);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 2);

	return res;
}

static enum target_state nulink_usb_state(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CHECK_MCU_STOP);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 4);

	if (res != ERROR_OK)
		return TARGET_UNKNOWN;

	if (!le_to_h_u32(h->databuf + 4 * 2))
		return TARGET_HALTED;
	else
		return TARGET_RUNNING;

	return TARGET_UNKNOWN;
}

static int nulink_usb_assert_srst(void *handle, int srst)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_assert_srst");

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
	h->cmdidx += 4;
	/* set reset type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, RESET_SYSRESETREQ);
	h->cmdidx += 4;
	/* set connect type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_NORMAL);
	h->cmdidx += 4;
	/* set extMode */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 4);

	return res;
}

static int nulink_usb_reset(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_reset");

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 4 * 4);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_RESET);
	h->cmdidx += 4;
	/* set reset type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, RESET_HW);
	h->cmdidx += 4;
	/* set connect type */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CONNECT_NORMAL);
	h->cmdidx += 4;
	/* set extMode */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 4);

	return res;
}

static int nulink_usb_run(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_run");

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_FREE_RUN);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 4);

	return res;
}

static int nulink_usb_halt(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_halt");

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_STOP_RUN);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 4);

	LOG_DEBUG("NULINK  stop_pc(0x%08x)",
		le_to_h_u32(h->databuf + 4));

	return res;
}

static int nulink_usb_step(void *handle)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_step");

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_STEP_RUN);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 4);

	LOG_DEBUG("NULINK pc(0x%08x)",
		le_to_h_u32(h->databuf + 4));

	return res;
}

static int nulink_usb_read_reg(void *handle, int num, uint32_t *val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_REG);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = 0xFF;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = 0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, num);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFFFF);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 2);

	*val = le_to_h_u32(h->databuf + 4 * 1);

	return res;
}

static int nulink_usb_write_reg(void *handle, int num, uint32_t val)
{
	int res;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	nulink_usb_init_buffer(handle, 8 + 12 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_REG);
	h->cmdidx += 4;
	/* Count of registers */
	h->cmdbuf[h->cmdidx] = 1;
	h->cmdidx += 1;
	/* Array of bool value (u8ReadOld) */
	h->cmdbuf[h->cmdidx] = 0x00;
	h->cmdidx += 1;
	/* Array of bool value (u8Verify) */
	h->cmdbuf[h->cmdidx] = 0x00;
	h->cmdidx += 1;
	/* ignore */
	h->cmdbuf[h->cmdidx] = 0;
	h->cmdidx += 1;
	/* u32Addr */
	h_u32_to_le(h->cmdbuf + h->cmdidx, num);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, val);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
	h->cmdidx += 4;

	res = nulink_usb_xfer(handle, h->databuf, 4 * 2);

	return res;
}

static int nulink_usb_read_mem8(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	unsigned aligned_addr, offset = 0;
	uint32_t bytes_remaining = 12;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_read_mem8: addr(0x%08x), len(%d)", addr, len);

	assert(handle != NULL);

	/* check whether data is word aligned */
	if (addr % 4) {
		aligned_addr = addr / 4;
		aligned_addr = aligned_addr * 4;
		offset = addr - aligned_addr;
		LOG_DEBUG("nulink_usb_read_mem8: address not aligned. addr(0x%08x)/aligned_addr(0x%08x)/offset(%d)",
			addr, aligned_addr, offset);

		addr = aligned_addr;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		if (len < 4)
			count = 1;
		else
			count = 2;

		nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = 0xFF;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = 0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFFFF);
			h->cmdidx += 4;
			/* proceed to the next one  */
			addr += 4;
		}

		res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		/* fill in the output buffer */
		for (i = 0; i < count; i++) {
			if (i == 0)
				memcpy(buffer, h->databuf + 4 + offset, len);
			else
				memcpy(buffer + 2 * i, h->databuf + 4 * (2 * i + 1), len - 2);
		}

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static int nulink_usb_write_mem8(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	unsigned aligned_addr, offset = 0;
	uint32_t bytes_remaining = 12;
	uint32_t u32bufferData;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_write_mem8: addr(0x%08x), len(%d)", addr, len);

	assert(handle != NULL);

	/* check whether data is word aligned */
	if (addr % 4) {
		aligned_addr = addr / 4;
		aligned_addr = aligned_addr * 4;
		offset = addr - aligned_addr;
		LOG_DEBUG("nulink_usb_write_mem8: address not aligned. addr(0x%08x)/aligned_addr(0x%08x)/offset(%d)",
			  addr, aligned_addr, offset);

		addr = aligned_addr;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		if (len < 4)
			count = 1;
		else
			count = 2;

		nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = 0x00;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = 0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			u32bufferData = buf_get_u32(buffer, 0, len * 8);
			u32bufferData = (u32bufferData << offset * 8);
			h_u32_to_le(h->cmdbuf + h->cmdidx, u32bufferData);
			h->cmdidx += 4;
			/* u32Mask */
			if (i == 0) {
				if (offset == 0) {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFF00);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFFFF00", i);
					} else {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF0000);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFF0000", i);
					}
				} else {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFF00FFFF);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFF00FFFF", i);

					} else {
						h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x0000FFFF);
						LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0x0000FFFF", i);
					}
				}
			} else {
				if (len == 4) {
					h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFF0000);
					LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0xFFFF0000", i);
				} else {
					h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
					LOG_DEBUG("nulink_usb_write_mem8: count(%d), mask: 0x00000000", i);
				}
			}
			h->cmdidx += 4;

			/* proceed to the next one */
			addr += 4;
			buffer += 4;
		}

		res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static int nulink_usb_read_mem32(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	uint32_t bytes_remaining = 12;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_ERROR("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		count = bytes_remaining / 4;

		nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = 0xFF;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = 0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0xFFFFFFFF);
			h->cmdidx += 4;
			/* proceed to the next one  */
			addr += 4;
		}

		res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		/* fill in the output buffer */
		for (i = 0; i < count; i++)
			memcpy(buffer + 4 * i, h->databuf + 4 * (2 * i + 1), 4);

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static int nulink_usb_write_mem32(void *handle, uint32_t addr, uint16_t len,
	const uint8_t *buffer)
{
	int res = ERROR_OK;
	unsigned i, count;
	uint32_t bytes_remaining = 12;
	uint32_t u32bufferData;
	struct nulink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_ERROR("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		count = bytes_remaining / 4;

		nulink_usb_init_buffer(handle, 8 + 12 * count);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_WRITE_RAM);
		h->cmdidx += 4;
		/* Count of registers */
		h->cmdbuf[h->cmdidx] = count;
		h->cmdidx += 1;
		/* Array of bool value (u8ReadOld) */
		h->cmdbuf[h->cmdidx] = 0x00;
		h->cmdidx += 1;
		/* Array of bool value (u8Verify) */
		h->cmdbuf[h->cmdidx] = 0x00;
		h->cmdidx += 1;
		/* ignore */
		h->cmdbuf[h->cmdidx] = 0;
		h->cmdidx += 1;

		for (i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			u32bufferData = buf_get_u32(buffer, 0, 32);
			h_u32_to_le(h->cmdbuf + h->cmdidx, u32bufferData);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, (unsigned long)0x00000000);
			h->cmdidx += 4;

			/* proceed to the next one */
			addr += 4;
			buffer += 4;
		}

		res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		if (len >= bytes_remaining)
			len -= bytes_remaining;
		else
			len = 0;
	}

	return res;
}

static uint32_t nulink_max_block_size(uint32_t tar_autoincr_block, uint32_t address)
{
	uint32_t max_tar_block = (tar_autoincr_block - ((tar_autoincr_block - 1) & address));

	if (max_tar_block == 0)
		max_tar_block = 4;

	return max_tar_block;
}

static int nulink_usb_read_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	struct nulink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	while (count) {

		bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

		if (count < bytes_remaining)
			bytes_remaining = count;

		/* the nulink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {

			/* When in jtag mode the nulink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the nulink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, according to ST they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr % 4) {

				uint32_t head_bytes = 4 - (addr % 4);
				retval = nulink_usb_read_mem8(handle, addr, head_bytes, buffer);
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining % 4)
				retval = nulink_usb_read_mem(handle, addr, 1, bytes_remaining, buffer);
			else
				retval = nulink_usb_read_mem32(handle, addr, bytes_remaining, buffer);
		} else
			retval = nulink_usb_read_mem8(handle, addr, bytes_remaining, buffer);

		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int nulink_usb_write_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	struct nulink_usb_handle_s *h = handle;

	if (addr < ARM_SRAM_BASE) {
		LOG_DEBUG("nulink_usb_write_mem: address below ARM_SRAM_BASE, not supported.\n");
		return retval;
	}

	/* calculate byte count */
	count *= size;

	while (count) {
		bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

		if (count < bytes_remaining)
			bytes_remaining = count;

		/* the nulink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {

			/* When in jtag mode the nulink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the nulink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, according to ST they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr % 4) {

				uint32_t head_bytes = 4 - (addr % 4);
				retval = nulink_usb_write_mem8(handle, addr, head_bytes, buffer);
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
		count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining % 4)
				retval = nulink_usb_write_mem(handle, addr, 1, bytes_remaining, buffer);
			else
				retval = nulink_usb_write_mem32(handle, addr, bytes_remaining, buffer);

		} else
			retval = nulink_usb_write_mem8(handle, addr, bytes_remaining, buffer);

		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int nulink_usb_override_target(const char *targetname)
{
	LOG_DEBUG("nulink_usb_override_target");

	return !strcmp(targetname, "cortex_m");
}

static int nulink_speed(void *handle, int khz, bool query)
{
	struct nulink_usb_handle_s *h = handle;
	unsigned long max_ice_clock = khz;

	LOG_DEBUG("nulink_speed: query(%d)", query);

	if (max_ice_clock > 12000)
		max_ice_clock = 12000;
	else if ((max_ice_clock == 3 * 512) || (max_ice_clock == 1500))
		max_ice_clock = 1500;
	else if (max_ice_clock >= 1000)
		max_ice_clock = max_ice_clock / 1000 * 1000;
	else
		max_ice_clock = max_ice_clock / 100 * 100;

	LOG_DEBUG("NULINK nulink_speed: %lu", max_ice_clock);

	if (!query) {
		nulink_usb_init_buffer(handle, 4 * 6);
		/* set command ID */
		h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_SET_CONFIG);
		h->cmdidx += 4;
		/* set max SWD clock */
		h_u32_to_le(h->cmdbuf + h->cmdidx, max_ice_clock);
		h->cmdidx += 4;
		/* chip type: NUC_CHIP_TYPE_GENERAL_V6M */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;
		/* IO voltage */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 5000);
		h->cmdidx += 4;
		/* If supply voltage to target or not */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
		h->cmdidx += 4;
		/* USB_FUNC_E: USB_FUNC_HID_BULK */
		h_u32_to_le(h->cmdbuf + h->cmdidx, 2);
		h->cmdidx += 4;

		nulink_usb_xfer(handle, h->databuf, 4 * 3);

		LOG_DEBUG("nulink_speed: h->hardwareConfig(%d)", h->hardwareConfig);
		if (h->hardwareConfig & 1) {
			LOG_INFO("NULINK target_voltage_mv[0](%04x), target_voltage_mv[1](%04x), target_voltage_mv[2](%04x), if_target_power_supplied(%d)",
				le_to_h_u32(h->databuf + 4 * 1),
				le_to_h_u32(h->databuf + 4 * 1) >> 16,
				le_to_h_u32(h->databuf + 4 * 2),
				(le_to_h_u32(h->databuf + 4 * 2) >> 16) & 1
				);
		}
	}

	return max_ice_clock;
}

static int nulink_usb_close(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_close");

	if (h && h->fd)
		jtag_libusb_close(h->fd);

	free(h);

	return ERROR_OK;
}

static int nulink_usb_open(struct hl_interface_param_s *param, void **fd)
{
	int err, retry_count = 1;
	struct nulink_usb_handle_s *h;

	LOG_DEBUG("nulink_usb_open");

	if (param->transport != HL_TRANSPORT_SWD)
		return TARGET_UNKNOWN;

	h = calloc(1, sizeof(struct nulink_usb_handle_s));

	if (h == 0) {
		LOG_ERROR("malloc failed");
		return ERROR_FAIL;
	}

	h->transport = param->transport;

	const char *serial = param->serial;

	do {
		if (jtag_libusb_open(param->vid, param->pid, serial, &h->fd, NULL) != ERROR_OK) {
			LOG_ERROR("open failed");
			return ERROR_FAIL;
		} else {
			LOG_DEBUG("jtag_libusb_open succeeded");
		}

		jtag_libusb_set_configuration(h->fd, 0);

		err = jtag_libusb_detach_kernel_driver(h->fd, 0);
		if (err != ERROR_OK) {
			LOG_ERROR("detach kernel driver failed(%d)", err);
			/* drivers might not binded, procced */
		} else {
			LOG_DEBUG("libusb_detach_kernel_driver succeeded");
		}

		err = jtag_libusb_claim_interface(h->fd, 0);
		if (err != ERROR_OK) {
			LOG_ERROR("claim interface failed(%d)", err);
			goto error_open;
		} else {
			LOG_DEBUG("jtag_libusb_claim_interface succeeded");
		}

		h->rx_ep = NULINK_RX_EP;
		h->tx_ep = NULINK_TX_EP;
		h->usbcmdidx = 0;
		h->hardwareConfig = 0;

		/* get the device version */
		err = nulink_usb_version(h);

		if (err == ERROR_OK) {
			break;
		} else {
			err = jtag_libusb_release_interface(h->fd, 0);
			if (err != ERROR_OK) {
				LOG_ERROR("release interface failed");
				goto error_open;
			}

			err = jtag_libusb_reset_device(h->fd);
			if (err != ERROR_OK) {
				LOG_ERROR("reset device failed");
				goto error_open;
			}

			jtag_libusb_close(h->fd);
			/*
			  Give the device one second to settle down and
			  reenumerate.
			 */
			usleep(1 * 1000 * 1000);
			if (0 == retry_count--)
				goto error_open;
		}
	} while (1);

	/* SWD clock rate : 1MHz */
	nulink_speed(h, 1000, false);

	/* get cpuid, so we can determine the max page size
	 * start with a safe default */
	h->max_mem_packet = (1 << 10);

	LOG_DEBUG("nulink_usb_open: we manually perform nulink_usb_reset");
	nulink_usb_write_debug_reg(h, 0xe000edf0, 0xa05f0001);
	nulink_usb_write_debug_reg(h, 0xe000edfc, 0x01000001);
	nulink_usb_write_debug_reg(h, 0xe000ed0c, 0x05fa0004);
	nulink_usb_reset(h);

	*fd = h;

	return ERROR_OK;

error_open:
	nulink_usb_close(h);

	return ERROR_FAIL;
}

struct hl_layout_api_s nulink_usb_layout_api = {

	.open = nulink_usb_open,

	.close = nulink_usb_close,

	.idcode = nulink_usb_idcode,

	.state = nulink_usb_state,

	.reset = nulink_usb_reset,

	.assert_srst = nulink_usb_assert_srst,

	.run = nulink_usb_run,

	.halt = nulink_usb_halt,

	.step = nulink_usb_step,

	.read_reg = nulink_usb_read_reg,

	.write_reg = nulink_usb_write_reg,

	.read_mem = nulink_usb_read_mem,

	.write_mem = nulink_usb_write_mem,

	.write_debug_reg = nulink_usb_write_debug_reg,

	.override_target = nulink_usb_override_target,

	.speed = nulink_speed,
};
