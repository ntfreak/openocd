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

#include <hidapi.h>

#define NULINK_READ_TIMEOUT  1000

#define NULINK_HID_MAX_SIZE   (64)
#define NULINK2_HID_MAX_SIZE   (1024)
#define V6M_MAX_COMMAND_LENGTH (NULINK_HID_MAX_SIZE - 2)
#define V7M_MAX_COMMAND_LENGTH (NULINK_HID_MAX_SIZE - 3)

#define NULINK2_USB_PID1  (0x5200)
#define NULINK2_USB_PID2  (0x5201)

struct nulink_usb_handle_s {
	hid_device *dev_handle;
	uint16_t max_packet_size;
	uint8_t usbcmdidx;
	uint8_t cmdidx;
	uint8_t cmdsize;
	uint8_t cmdbuf[NULINK2_HID_MAX_SIZE + 1];
	uint8_t tempbuf[NULINK2_HID_MAX_SIZE];
	uint8_t databuf[NULINK2_HID_MAX_SIZE];
	uint32_t max_mem_packet;
	uint16_t hardware_config; /* bit 0: 1:Nu-Link-Pro, 0:Nu-Link */

	int (*xfer)(void *handle, uint8_t *buf, int size);
	void (*init_buffer)(void *handle, uint32_t size);
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

#define ARM_SRAM_BASE				0x20000000UL

#define HARDWARE_CONFIG_NULINKPRO	1
#define HARDWARE_CONFIG_NULINK2		2

enum nulink_reset {
	RESET_AUTO = 0,
	RESET_HW = 1,
	RESET_SYSRESETREQ = 2,
	RESET_VECTRESET = 3,
	RESET_FAST_RESCUE = 4, /* Rescue and erase the chip, need very fast speed */
};

enum nulink_connect {
	CONNECT_NORMAL = 0,      /* Support all reset method */
	CONNECT_PRE_RESET = 1,   /* Support all reset method */
	CONNECT_UNDER_RESET = 2, /* Support all reset method */
	CONNECT_NONE = 3,        /* Support RESET_HW, (RESET_AUTO = RESET_HW) */
	CONNECT_DISCONNECT = 4,  /* Support RESET_NONE, (RESET_AUTO = RESET_NONE) */
	CONNECT_ICP_MODE = 5     /* Support NUC505 ICP mode*/
};

static int nulink_usb_xfer_rw(void *handle, uint8_t *buf)
{
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

	int ret = hid_write(h->dev_handle, h->cmdbuf, h->max_packet_size + 1);
	if (ret < 0) {
		LOG_ERROR("hid_write");
		return ERROR_FAIL;
	}

	ret = hid_read_timeout(h->dev_handle, buf, h->max_packet_size, NULINK_READ_TIMEOUT);
	if (ret < 0) {
		LOG_ERROR("hid_read_timeout");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int nulink1_usb_xfer(void *handle, uint8_t *buf, int size)
{
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

	int err = nulink_usb_xfer_rw(h, h->tempbuf);

	memcpy(buf, h->tempbuf + 2, V6M_MAX_COMMAND_LENGTH);

	return err;
}

static int nulink2_usb_xfer(void *handle, uint8_t *buf, int size)
{
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

	int err = nulink_usb_xfer_rw(h, h->tempbuf);

	memcpy(buf, h->tempbuf + 3, V7M_MAX_COMMAND_LENGTH);

	return err;
}

static void nulink1_usb_init_buffer(void *handle, uint32_t size)
{
	struct nulink_usb_handle_s *h = handle;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, h->max_packet_size + 1);
	memset(h->tempbuf, 0, h->max_packet_size);
	memset(h->databuf, 0, h->max_packet_size);

	h->cmdbuf[0] = 0; /* report number */
	h->cmdbuf[1] = ++h->usbcmdidx & 0x7F;
	h->cmdbuf[2] = size;
	h->cmdidx += 3;
}

static void nulink2_usb_init_buffer(void *handle, uint32_t size)
{
	struct nulink_usb_handle_s *h = handle;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, h->max_packet_size + 1);
	memset(h->tempbuf, 0, h->max_packet_size);
	memset(h->databuf, 0, h->max_packet_size);

	h->cmdbuf[0] = 0; /* report number */
	h->cmdbuf[1] = ++h->usbcmdidx & 0x7F;
	h_u16_to_le(h->cmdbuf + 2, size);
	h->cmdidx += 4;
}

static inline int nulink_usb_xfer(void *handle, uint8_t *buf, int size)
{
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

	return h->xfer(handle, buf, size);
}

static inline void nulink_usb_init_buffer(void *handle, uint32_t size)
{
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

	h->init_buffer(handle, size);
}

static int nulink_usb_version(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_version");

	assert(handle);

	nulink_usb_init_buffer(handle, V6M_MAX_COMMAND_LENGTH);

	memset(h->cmdbuf + h->cmdidx, 0xFF, V6M_MAX_COMMAND_LENGTH);
	h->cmdbuf[h->cmdidx + 4] = 0xA1; /* host_rev_num: 6561 */;
	h->cmdbuf[h->cmdidx + 5] = 0x19;

	int res = nulink_usb_xfer(handle, h->databuf, h->cmdsize);
	if (res != ERROR_OK)
		return res;

	LOG_INFO("Nu-Link firmware_version %" PRIu32 ", product_id (0x%08" PRIx32 ")",
			 le_to_h_u32(h->databuf),
			 le_to_h_u32(h->databuf + 4 * 1));

	const bool is_nulinkpro = !!(le_to_h_u32(h->databuf + 4 * 2) & 1);
	if (is_nulinkpro) {
		LOG_INFO("Adapter is Nu-Link-Pro, target_voltage_mv(%" PRIu16 "), usb_voltage_mv(%" PRIu16 ")",
				 le_to_h_u16(h->databuf + 4 * 3 + 0),
				 le_to_h_u16(h->databuf + 4 * 3 + 2));

		h->hardware_config |= HARDWARE_CONFIG_NULINKPRO;
	} else {
		LOG_INFO("Adapter is Nu-Link");
	}

	return ERROR_OK;
}

static int nulink_usb_idcode(void *handle, uint32_t *idcode)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_idcode");

	assert(handle);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CHECK_ID);
	h->cmdidx += 4;

	int res = nulink_usb_xfer(handle, h->databuf, 4 * 2);
	if (res != ERROR_OK)
		return res;

	*idcode = le_to_h_u32(h->databuf + 4 * 1);

	LOG_INFO("IDCODE: 0x%08" PRIX32, *idcode);

	return ERROR_OK;
}

static int nulink_usb_write_debug_reg(void *handle, uint32_t addr, uint32_t val)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_write_debug_reg 0x%08" PRIX32 "0x%08" PRIX32, addr, val);

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
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0x00000000UL);
	h->cmdidx += 4;

	return nulink_usb_xfer(handle, h->databuf, 4 * 2);
}

static enum target_state nulink_usb_state(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_CHECK_MCU_STOP);
	h->cmdidx += 4;

	int res = nulink_usb_xfer(handle, h->databuf, 4 * 4);
	if (res != ERROR_OK)
		return TARGET_UNKNOWN;

	if (!le_to_h_u32(h->databuf + 4 * 2))
		return TARGET_HALTED;
	else
		return TARGET_RUNNING;
}

static int nulink_usb_assert_srst(void *handle, int srst)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_assert_srst");

	assert(handle);

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

	return nulink_usb_xfer(handle, h->databuf, 4 * 4);
}

static int nulink_usb_reset(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_reset");

	assert(handle);

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

	return nulink_usb_xfer(handle, h->databuf, 4 * 4);
}

static int nulink_usb_run(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_run");

	assert(handle);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_FREE_RUN);
	h->cmdidx += 4;

	return nulink_usb_xfer(handle, h->databuf, 4 * 4);
}

static int nulink_usb_halt(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_halt");

	assert(handle);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_STOP_RUN);
	h->cmdidx += 4;

	int res = nulink_usb_xfer(handle, h->databuf, 4 * 4);

	LOG_DEBUG("Nu-Link stop_pc 0x%08" PRIx32, le_to_h_u32(h->databuf + 4));

	return res;
}

static int nulink_usb_step(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_step");

	assert(handle);

	nulink_usb_init_buffer(handle, 4 * 1);
	/* set command ID */
	h_u32_to_le(h->cmdbuf + h->cmdidx, CMD_MCU_STEP_RUN);
	h->cmdidx += 4;

	int res = nulink_usb_xfer(handle, h->databuf, 4 * 4);

	LOG_DEBUG("Nu-Link pc 0x%08" PRIx32, le_to_h_u32(h->databuf + 4));

	return res;
}

static int nulink_usb_read_reg(void *handle, unsigned int regsel, uint32_t *val)
{
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

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
	h_u32_to_le(h->cmdbuf + h->cmdidx, regsel);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFFUL);
	h->cmdidx += 4;

	int res = nulink_usb_xfer(handle, h->databuf, 4 * 2);

	*val = le_to_h_u32(h->databuf + 4 * 1);

	return res;
}

static int nulink_usb_write_reg(void *handle, unsigned int regsel, uint32_t val)
{
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

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
	h_u32_to_le(h->cmdbuf + h->cmdidx, regsel);
	h->cmdidx += 4;
	/* u32Data */
	h_u32_to_le(h->cmdbuf + h->cmdidx, val);
	h->cmdidx += 4;
	/* u32Mask */
	h_u32_to_le(h->cmdbuf + h->cmdidx, 0x00000000UL);
	h->cmdidx += 4;

	return nulink_usb_xfer(handle, h->databuf, 4 * 2);
}

static int nulink_usb_read_mem8(void *handle, uint32_t addr, uint16_t len,
		uint8_t *buffer)
{
	int res = ERROR_OK;
	uint32_t offset = 0;
	uint32_t bytes_remaining = 12;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_read_mem8: addr 0x%08" PRIx32 ", len %" PRId16, addr, len);

	assert(handle);

	/* check whether data is word aligned */
	if (addr % 4) {
		uint32_t aligned_addr = addr / 4;
		aligned_addr = aligned_addr * 4;
		offset = addr - aligned_addr;
		LOG_DEBUG("nulink_usb_read_mem8: unaligned address addr 0x%08" PRIx32
				"/aligned addr 0x%08" PRIx32 "offset %" PRIu32,
				addr, aligned_addr, offset);

		addr = aligned_addr;
	}

	while (len) {
		unsigned int count;

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

		for (unsigned int i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFFUL);
			h->cmdidx += 4;
			/* proceed to the next one  */
			addr += 4;
		}

		res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);
		if (res != ERROR_OK)
			break;

		/* fill in the output buffer */
		for (unsigned int i = 0; i < count; i++) {
			if (i == 0)
				memcpy(buffer, h->databuf + 4 + offset, len);
			else
				memcpy(buffer + 2 * i, h->databuf + 4 * (2 * i + 1), len - 2);
		}

		if (len >= bytes_remaining)
			len -= bytes_remaining;
	}

	return res;
}

static int nulink_usb_write_mem8(void *handle, uint32_t addr, uint16_t len,
		const uint8_t *buffer)
{
	int res = ERROR_OK;
	uint32_t offset = 0;
	uint32_t bytes_remaining = 12;
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_write_mem8: addr 0x%08" PRIx32 ", len %" PRIu16, addr, len);

	assert(handle);

	/* check whether data is word aligned */
	if (addr % 4) {
		uint32_t aligned_addr = addr / 4;
		aligned_addr = aligned_addr * 4;
		offset = addr - aligned_addr;
		LOG_DEBUG("nulink_usb_write_mem8: address not aligned. addr(0x%08" PRIx32
				")/aligned_addr(0x%08" PRIx32 ")/offset(%" PRIu32 ")",
				addr, aligned_addr, offset);

		addr = aligned_addr;
	}

	while (len) {
		unsigned int count;

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

		for (unsigned int i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			uint32_t u32buffer = buf_get_u32(buffer, 0, len * 8);
			u32buffer = (u32buffer << offset * 8);
			h_u32_to_le(h->cmdbuf + h->cmdidx, u32buffer);
			h->cmdidx += 4;
			/* u32Mask */
			if (i == 0) {
				if (offset == 0) {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFF00UL);
						LOG_DEBUG("nulink_usb_write_mem8: count(%u), mask: 0xFFFFFF00", i);
					} else {
						h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFF0000UL);
						LOG_DEBUG("nulink_usb_write_mem8: count(%u), mask: 0xFFFF0000", i);
					}
				} else {
					if (len == 1) {
						h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFF00FFFFUL);
						LOG_DEBUG("nulink_usb_write_mem8: count(%u), mask: 0xFF00FFFF", i);

					} else {
						h_u32_to_le(h->cmdbuf + h->cmdidx, 0x0000FFFFUL);
						LOG_DEBUG("nulink_usb_write_mem8: count(%u), mask: 0x0000FFFF", i);
					}
				}
			} else {
				if (len == 4) {
					h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFF0000UL);
					LOG_DEBUG("nulink_usb_write_mem8: count(%u), mask: 0xFFFF0000", i);
				} else {
					h_u32_to_le(h->cmdbuf + h->cmdidx, 0x00000000UL);
					LOG_DEBUG("nulink_usb_write_mem8: count(%u), mask: 0x00000000", i);
				}
			}
			h->cmdidx += 4;

			/* proceed to the next one */
			addr += 4;
			buffer += 4;
		}

		res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);
		if (res != ERROR_OK)
			break;

		if (len >= bytes_remaining)
			len -= bytes_remaining;
	}

	return res;
}

static int nulink_usb_read_mem32(void *handle, uint32_t addr, uint16_t len,
		uint8_t *buffer)
{
	int res = ERROR_OK;
	uint32_t bytes_remaining = 12;
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_ERROR("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		unsigned int count = bytes_remaining / 4;

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

		for (unsigned int i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0xFFFFFFFFUL);
			h->cmdidx += 4;
			/* proceed to the next one  */
			addr += 4;
		}

		res = nulink_usb_xfer(handle, h->databuf, 4 * count * 2);

		/* fill in the output buffer */
		for (unsigned int i = 0; i < count; i++) {
			memcpy(buffer, h->databuf + 4 * (2 * i + 1), 4);
			buffer += 4;
		}

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
	uint32_t bytes_remaining = 12;
	struct nulink_usb_handle_s *h = handle;

	assert(handle);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_ERROR("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	while (len) {
		if (len < bytes_remaining)
			bytes_remaining = len;

		unsigned int count = bytes_remaining / 4;

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

		for (unsigned int i = 0; i < count; i++) {
			/* u32Addr */
			h_u32_to_le(h->cmdbuf + h->cmdidx, addr);
			h->cmdidx += 4;
			/* u32Data */
			uint32_t u32buffer = buf_get_u32(buffer, 0, 32);
			h_u32_to_le(h->cmdbuf + h->cmdidx, u32buffer);
			h->cmdidx += 4;
			/* u32Mask */
			h_u32_to_le(h->cmdbuf + h->cmdidx, 0x00000000);
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
	struct nulink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	while (count) {
		uint32_t bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

		if (count < bytes_remaining)
			bytes_remaining = count;

		if (bytes_remaining >= 4)
			size = 4;

		/* the nulink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {
			/* When in jtag mode the nulink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the nulink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, they do single
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
		} else {
			retval = nulink_usb_read_mem8(handle, addr, bytes_remaining, buffer);
		}

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
	struct nulink_usb_handle_s *h = handle;

	if (addr < ARM_SRAM_BASE) {
		LOG_DEBUG("nulink_usb_write_mem: address below ARM_SRAM_BASE, not supported.\n");
		return retval;
	}

	/* calculate byte count */
	count *= size;

	while (count) {
		uint32_t bytes_remaining = nulink_max_block_size(h->max_mem_packet, addr);

		if (count < bytes_remaining)
			bytes_remaining = count;

		if (bytes_remaining >= 4)
			size = 4;

		/* the nulink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {
			/* When in jtag mode the nulink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the nulink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, do single
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

		} else {
			retval = nulink_usb_write_mem8(handle, addr, bytes_remaining, buffer);
		}

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

	LOG_DEBUG("nulink_speed: query %s", query ? "yes" : "no");

	if (max_ice_clock > 12000)
		max_ice_clock = 12000;
	else if ((max_ice_clock == 3 * 512) || (max_ice_clock == 1500))
		max_ice_clock = 1500;
	else if (max_ice_clock >= 1000)
		max_ice_clock = max_ice_clock / 1000 * 1000;
	else
		max_ice_clock = max_ice_clock / 100 * 100;

	LOG_DEBUG("Nu-Link nulink_speed: %lu", max_ice_clock);

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

		LOG_DEBUG("nulink_speed: h->hardware_config(%" PRId16 ")", h->hardware_config);
		if (h->hardware_config & HARDWARE_CONFIG_NULINKPRO)
			LOG_INFO("Nu-Link target_voltage_mv[0](%04" PRIx16 "), target_voltage_mv[1](%04" PRIx16
				"), target_voltage_mv[2](%04" PRIx16 "), if_target_power_supplied(%d)",
				le_to_h_u16(h->databuf + 4 * 1 + 0),
				le_to_h_u16(h->databuf + 4 * 1 + 2),
				le_to_h_u16(h->databuf + 4 * 2 + 0),
				le_to_h_u16(h->databuf + 4 * 2 + 2) & 1);
	}

	return max_ice_clock;
}

static int nulink_usb_close(void *handle)
{
	struct nulink_usb_handle_s *h = handle;

	LOG_DEBUG("nulink_usb_close");

	if (h && h->dev_handle)
		hid_close(h->dev_handle);

	free(h);

	hid_exit();

	return ERROR_OK;
}

static int nulink_usb_open(struct hl_interface_param_s *param, void **fd)
{
	struct hid_device_info *devs, *cur_dev;
	uint16_t target_vid = 0;
	uint16_t target_pid = 0;
	wchar_t *target_serial = NULL;

	LOG_DEBUG("nulink_usb_open");

	if (param->transport != HL_TRANSPORT_SWD)
		return TARGET_UNKNOWN;

	if (!param->vid[0] && !param->pid[0]) {
		LOG_ERROR("Missing vid/pid");
		return ERROR_FAIL;
	}

	if (hid_init() != 0) {
		LOG_ERROR("unable to open HIDAPI");
		return ERROR_FAIL;
	}

	struct nulink_usb_handle_s *h = calloc(1, sizeof(*h));
	if (!h) {
		LOG_ERROR("Out of memory");
		goto error_open;
	}

	if (param->serial) {
		size_t len = mbstowcs(NULL, param->serial, 0);

		target_serial = calloc(len + 1, sizeof(wchar_t));
		if (!target_serial) {
			LOG_ERROR("Out of memory");
			goto error_open;
		}

		if (mbstowcs(target_serial, param->serial, len + 1) == (size_t)(-1)) {
			LOG_WARNING("unable to convert serial");
			free(target_serial);
			target_serial = NULL;
		}
	}

	devs = hid_enumerate(0, 0);
	cur_dev = devs;
	while (cur_dev) {
		bool found = false;

		for (unsigned int i = 0; param->vid[i] || param->pid[i]; i++) {
			if (param->vid[i] == cur_dev->vendor_id && param->pid[i] == cur_dev->product_id) {
				found = true;
				break;
			}
		}

		if (found) {
			if (!target_serial)
				break;
			if (cur_dev->serial_number && wcscmp(target_serial, cur_dev->serial_number) == 0)
				break;
		}

		cur_dev = cur_dev->next;
	}
	if (cur_dev) {
		target_vid = cur_dev->vendor_id;
		target_pid = cur_dev->product_id;
	}

	hid_free_enumeration(devs);

	if (target_vid == 0 && target_pid == 0) {
		LOG_ERROR("unable to find Nu-Link");
		goto error_open;
	}

	hid_device *dev = hid_open(target_vid, target_pid, target_serial);
	if (!dev) {
		LOG_ERROR("unable to open Nu-Link device 0x%" PRIx16 ":0x%" PRIx16, target_vid, target_pid);
		goto error_open;
	}

	h->dev_handle = dev;
	h->usbcmdidx = 0;

	switch (target_pid) {
	case NULINK2_USB_PID1:
	case NULINK2_USB_PID2:
		h->hardware_config = HARDWARE_CONFIG_NULINK2;
		h->max_packet_size = NULINK2_HID_MAX_SIZE;
		h->init_buffer = nulink2_usb_init_buffer;
		h->xfer = nulink2_usb_xfer;
		break;
	default:
		h->hardware_config = 0;
		h->max_packet_size = NULINK_HID_MAX_SIZE;
		h->init_buffer = nulink1_usb_init_buffer;
		h->xfer = nulink1_usb_xfer;
		break;
	}

	/* get the device version */
	h->cmdsize = 4 * 5;
	int err = nulink_usb_version(h);
	if (err != ERROR_OK) {
		LOG_DEBUG("nulink_usb_version failed with cmdSize(4 * 5)");
		h->cmdsize = 4 * 6;
		err = nulink_usb_version(h);
		if (err != ERROR_OK)
			LOG_DEBUG("nulink_usb_version failed with cmdSize(4 * 6)");
	}

	/* SWD clock rate : 1MHz */
	nulink_speed(h, 1000, false);

	/* get cpuid, so we can determine the max page size
	 * start with a safe default */
	h->max_mem_packet = (1 << 10);

	LOG_DEBUG("nulink_usb_open: we manually perform nulink_usb_reset");
	nulink_usb_reset(h);

	*fd = h;

	free(target_serial);
	return ERROR_OK;

error_open:
	nulink_usb_close(h);
	free(target_serial);

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
