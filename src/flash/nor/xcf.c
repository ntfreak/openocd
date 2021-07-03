/***************************************************************************
 *   Copyright (C) 2016 by Uladzimir Pylinski aka barthess                 *
 *   barthess@yandex.ru                                                    *
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

#include <string.h>

#include "imp.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

#define SECTOR_ERASE_TIMEOUT_MS         (35 * 1000)

#define XCF_PAGE_SIZE                   32
#define XCF_DATA_SECTOR_SIZE            (1024 * 1024)

#define ID_XCF01S                       0x05044093
#define ID_XCF02S                       0x05045093
#define ID_XCF04S                       0x05046093
#define ID_XCF08P                       0x05057093
#define ID_XCF16P                       0x05058093
#define ID_XCF32P                       0x05059093
#define ID_MEANINGFUL_MASK              0x0FFFFFFF

static const char * const xcf_name_list[] = {
	"XCF08P",
	"XCF16P",
	"XCF32P",
	"unknown"
};

struct xcf_priv {
	bool probed;
};

struct xcf_status {
	bool isc_error;		/* false == OK, true == error */
	bool prog_error;	/* false == OK, true == error */
	bool prog_busy;		/* false == idle, true == busy */
	bool isc_mode;		/* false == normal mode, true == ISC mode */
};

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */
static const uint8_t cmd_bypass[2]              = {0xFF, 0xFF};

static const uint8_t cmd_isc_address_shift[2]   = {0xEB, 0x00};
static const uint8_t cmd_isc_data_shift[2]      = {0xED, 0x00};
static const uint8_t cmd_isc_disable[2]         = {0xF0, 0x00};
static const uint8_t cmd_isc_enable[2]          = {0xE8, 0x00};
static const uint8_t cmd_isc_erase[2]           = {0xEC, 0x00};
static const uint8_t cmd_isc_program[2]         = {0xEA, 0x00};

static const uint8_t cmd_xsc_blank_check[2]     = {0x0D, 0x00};
static const uint8_t cmd_xsc_config[2]          = {0xEE, 0x00};
static const uint8_t cmd_xsc_data_btc[2]        = {0xF2, 0x00};
static const uint8_t cmd_xsc_data_ccb[2]        = {0x0C, 0x00};
static const uint8_t cmd_xsc_data_done[2]       = {0x09, 0x00};
static const uint8_t cmd_xsc_data_sucr[2]       = {0x0E, 0x00};
static const uint8_t cmd_xsc_data_wrpt[2]       = {0xF7, 0x00};
static const uint8_t cmd_xsc_op_status[2]       = {0xE3, 0x00};
static const uint8_t cmd_xsc_read[2]            = {0xEF, 0x00};
static const uint8_t cmd_xsc_unlock[2]          = {0x55, 0xAA};

/*
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 */

static const char *product_name(const struct flash_bank *bank)
{

	switch (bank->target->tap->idcode & ID_MEANINGFUL_MASK) {
		case ID_XCF08P:
			return xcf_name_list[0];
		case ID_XCF16P:
			return xcf_name_list[1];
		case ID_XCF32P:
			return xcf_name_list[2];
		default:
			return xcf_name_list[3];
	}
}

static void fill_sector_table(struct flash_bank *bank)
{
	/* Note: is_erased and is_protected fields must be set here to an unknown
	 * state, they will be correctly filled from other API calls. */

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].is_erased              = -1;
		bank->sectors[i].is_protected   = -1;
	}
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size   = XCF_DATA_SECTOR_SIZE;
		bank->sectors[i].offset = i * XCF_DATA_SECTOR_SIZE;
	}

	bank->size = bank->num_sectors * XCF_DATA_SECTOR_SIZE;
}

static struct xcf_status read_status(struct flash_bank *bank)
{
	struct xcf_status ret;
	uint8_t irdata[2];
	struct scan_field scan;

	scan.check_mask = NULL;
	scan.check_value = NULL;
	scan.num_bits = 16;
	scan.out_value = cmd_bypass;
	scan.in_value = irdata;

	jtag_add_ir_scan(bank->target->tap, &scan, TAP_IDLE);
	jtag_execute_queue();

	ret.isc_error   = ((irdata[0] >> 7) & 3) == 0b01;
	ret.prog_error  = ((irdata[0] >> 5) & 3) == 0b01;
	ret.prog_busy   = ((irdata[0] >> 4) & 1) == 0;
	ret.isc_mode    = ((irdata[0] >> 3) & 1) == 1;

	return ret;
}

static int isc_enter(struct flash_bank *bank)
{

	struct xcf_status status = read_status(bank);

	if (true == status.isc_mode)
		return ERROR_OK;
	else {
		struct scan_field scan;

		scan.check_mask = NULL;
		scan.check_value = NULL;
		scan.num_bits = 16;
		scan.out_value = cmd_isc_enable;
		scan.in_value = NULL;

		jtag_add_ir_scan(bank->target->tap, &scan, TAP_IDLE);
		jtag_execute_queue();

		status = read_status(bank);
		if (!status.isc_mode) {
			LOG_ERROR("*** XCF: FAILED to enter ISC mode");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		return ERROR_OK;
	}
}

static int isc_leave(struct flash_bank *bank)
{

	struct xcf_status status = read_status(bank);

	if (!status.isc_mode)
		return ERROR_OK;
	else {
		struct scan_field scan;

		scan.check_mask = NULL;
		scan.check_value = NULL;
		scan.num_bits = 16;
		scan.out_value = cmd_isc_disable;
		scan.in_value = NULL;

		jtag_add_ir_scan(bank->target->tap, &scan, TAP_IDLE);
		jtag_execute_queue();
		alive_sleep(1);	/* device needs 50 uS to leave ISC mode */

		status = read_status(bank);
		if (status.isc_mode) {
			LOG_ERROR("*** XCF: FAILED to leave ISC mode");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		return ERROR_OK;
	}
}

static int sector_state(uint8_t wrpt, int sector)
{
	if (((wrpt >> sector) & 1) == 1)
		return 0;
	else
		return 1;
}

static uint8_t fill_select_block(unsigned int first, unsigned int last)
{
	uint8_t ret = 0;
	for (unsigned int i = first; i <= last; i++)
		ret |= 1 << i;
	return ret;
}

static int isc_read_register(struct flash_bank *bank, const uint8_t *cmd,
	uint8_t *data_buf, int num_bits)
{
	struct scan_field scan;

	scan.check_mask = NULL;
	scan.check_value = NULL;
	scan.out_value = cmd;
	scan.in_value = NULL;
	scan.num_bits = 16;
	jtag_add_ir_scan(bank->target->tap, &scan, TAP_DRSHIFT);

	scan.out_value = NULL;
	scan.in_value = data_buf;
	scan.num_bits = num_bits;
	jtag_add_dr_scan(bank->target->tap, 1, &scan, TAP_IDLE);

	return jtag_execute_queue();
}

static int isc_wait_erase_program(struct flash_bank *bank, int64_t timeout_ms)
{

	uint8_t isc_default;
	int64_t t0 = timeval_ms();
	int64_t dt;

	do {
		isc_read_register(bank, cmd_xsc_op_status, &isc_default, 8);
		if (((isc_default >> 2) & 1) == 1)
			return ERROR_OK;
		dt = timeval_ms() - t0;
	} while (dt <= timeout_ms);
	return ERROR_FLASH_OPERATION_FAILED;
}

/*
 * helper function for procedures without program jtag command at the end
 */
static int isc_set_register(struct flash_bank *bank, const uint8_t *cmd,
	const uint8_t *data_buf, int num_bits, int64_t timeout_ms)
{
	struct scan_field scan;

	scan.check_mask = NULL;
	scan.check_value = NULL;
	scan.num_bits = 16;
	scan.out_value = cmd;
	scan.in_value = NULL;
	jtag_add_ir_scan(bank->target->tap, &scan, TAP_DRSHIFT);

	scan.num_bits = num_bits;
	scan.out_value = data_buf;
	scan.in_value = NULL;
	jtag_add_dr_scan(bank->target->tap, 1, &scan, TAP_IDLE);

	if (timeout_ms == 0)
		return jtag_execute_queue();
	else
		return isc_wait_erase_program(bank, timeout_ms);
}

/*
 * helper function for procedures required program jtag command at the end
 */
static int isc_program_register(struct flash_bank *bank, const uint8_t *cmd,
	const uint8_t *data_buf, int num_bits, int64_t timeout_ms)
{
	struct scan_field scan;

	scan.check_mask = NULL;
	scan.check_value = NULL;
	scan.num_bits = 16;
	scan.out_value = cmd;
	scan.in_value = NULL;
	jtag_add_ir_scan(bank->target->tap, &scan, TAP_DRSHIFT);

	scan.num_bits = num_bits;
	scan.out_value = data_buf;
	scan.in_value = NULL;
	jtag_add_dr_scan(bank->target->tap, 1, &scan, TAP_IRSHIFT);

	scan.num_bits = 16;
	scan.out_value = cmd_isc_program;
	scan.in_value = NULL;
	jtag_add_ir_scan(bank->target->tap, &scan, TAP_IDLE);

	if (timeout_ms == 0)
		return jtag_execute_queue();
	else
		return isc_wait_erase_program(bank, timeout_ms);
}

static int isc_clear_protect(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	uint8_t select_block[3] = {0x0, 0x0, 0x0};
	select_block[0] = fill_select_block(first, last);
	return isc_set_register(bank, cmd_xsc_unlock, select_block, 24, 0);
}

static int isc_set_protect(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	uint8_t wrpt[2] = {0xFF, 0xFF};
	for (unsigned int i = first; i <= last; i++)
		wrpt[0] &= ~(1 << i);

	return isc_program_register(bank, cmd_xsc_data_wrpt, wrpt, 16, 0);
}

static int isc_erase_sectors(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	uint8_t select_block[3] = {0, 0, 0};
	select_block[0] = fill_select_block(first, last);
	int64_t timeout = SECTOR_ERASE_TIMEOUT_MS * (last - first + 1);
	return isc_set_register(bank, cmd_isc_erase, select_block, 24, timeout);
}

static int isc_adr_shift(struct flash_bank *bank, int adr)
{
	uint8_t adr_buf[3];
	h_u24_to_le(adr_buf, adr);
	return isc_set_register(bank, cmd_isc_address_shift, adr_buf, 24, 0);
}

static int isc_program_data_page(struct flash_bank *bank, const uint8_t *page_buf)
{
	return isc_program_register(bank, cmd_isc_data_shift, page_buf, 8 * XCF_PAGE_SIZE, 100);
}

static void isc_data_read_out(struct flash_bank *bank, uint8_t *buffer, uint32_t count)
{

	struct scan_field scan;

	/* Do not change this code with isc_read_register() call because it needs
	 * transition to IDLE state before data retrieving. */
	scan.check_mask = NULL;
	scan.check_value = NULL;
	scan.num_bits = 16;
	scan.out_value = cmd_xsc_read;
	scan.in_value = NULL;
	jtag_add_ir_scan(bank->target->tap, &scan, TAP_IDLE);

	scan.num_bits = 8 * count;
	scan.out_value = NULL;
	scan.in_value = buffer;
	jtag_add_dr_scan(bank->target->tap, 1, &scan, TAP_IDLE);

	jtag_execute_queue();
}

static int isc_set_data_done(struct flash_bank *bank, int sector)
{
	uint8_t done = 0xFF;
	done &= ~(1 << sector);
	return isc_program_register(bank, cmd_xsc_data_done, &done, 8, 100);
}

static void flip_u8(uint8_t *out, const uint8_t *in, int len)
{
	for (int i = 0; i < len; i++)
		out[i] = flip_u32(in[i], 8);
}

/*
 * Xilinx bin file contains simple fixed header for automatic bus width detection:
 * 16 bytes of 0xFF
 * 4 byte sync word 0xAA995566 or (bit reversed) 0x5599AA66 in MSC file
 *
 * Function presumes need of bit reversing if it can not exactly detects
 * the opposite.
 */
static bool need_bit_reverse(const uint8_t *buffer)
{
	const size_t L = 20;
	uint8_t reference[L];
	memset(reference, 0xFF, 16);
	reference[16] = 0x55;
	reference[17] = 0x99;
	reference[18] = 0xAA;
	reference[19] = 0x66;

	if (memcmp(reference, buffer, L) == 0)
		return false;
	else
		return true;
}

/*
 * The page address to be programmed is determined by loading the
 * internal ADDRESS Register using an ISC_ADDRESS_SHIFT instruction sequence.
 * The page address automatically increments to the next 256-bit
 * page address after each programming sequence until the last address
 * in the 8 Mb block is reached. To continue programming the next block,
 * the next 8 Mb block's starting address must be loaded into the
 * internal ADDRESS register.
 */
static int read_write_data(struct flash_bank *bank, const uint8_t *w_buffer,
	uint8_t *r_buffer, bool write_flag, uint32_t offset, uint32_t count)
{
	int dbg_count = count;
	int dbg_written = 0;
	int ret = ERROR_OK;
	uint8_t *page_buf = malloc(XCF_PAGE_SIZE);
	bool revbit = true;
	isc_enter(bank);

	if (offset % XCF_PAGE_SIZE != 0) {
		ret = ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		goto EXIT;
	}

	if ((offset + count) > (bank->num_sectors * XCF_DATA_SECTOR_SIZE)) {
		ret = ERROR_FLASH_DST_OUT_OF_BANK;
		goto EXIT;
	}

	if ((write_flag) && (offset == 0) && (count >= XCF_PAGE_SIZE))
		revbit = need_bit_reverse(w_buffer);

	while (count > 0) {
		uint32_t sector_num = offset / XCF_DATA_SECTOR_SIZE;
		uint32_t sector_offset = offset - sector_num * XCF_DATA_SECTOR_SIZE;
		uint32_t sector_bytes = XCF_DATA_SECTOR_SIZE - sector_offset;
		if (count < sector_bytes)
			sector_bytes = count;
		isc_adr_shift(bank, offset);
		offset += sector_bytes;
		count -= sector_bytes;

		if (write_flag) {
			while (sector_bytes > 0) {
				int len;

				if (sector_bytes < XCF_PAGE_SIZE) {
					len = sector_bytes;
					memset(page_buf, 0xFF, XCF_PAGE_SIZE);
				} else
					len = XCF_PAGE_SIZE;

				if (revbit)
					flip_u8(page_buf, w_buffer, len);
				else
					memcpy(page_buf, w_buffer, len);

				w_buffer += len;
				sector_bytes -= len;
				ret = isc_program_data_page(bank, page_buf);
				if (ret != ERROR_OK)
					goto EXIT;
				else {
					LOG_DEBUG("written %d bytes from %d", dbg_written, dbg_count);
					dbg_written += len;
				}
			}
		} else {
			isc_data_read_out(bank, r_buffer, sector_bytes);
			flip_u8(r_buffer, r_buffer, sector_bytes);
			r_buffer += sector_bytes;
		}
	}

	/* Set 'done' flags for all data sectors because driver supports
	 * only single revision. */
	if (write_flag) {
		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			ret = isc_set_data_done(bank, i);
			if (ret != ERROR_OK)
				goto EXIT;
		}
	}

EXIT:
	free(page_buf);
	isc_leave(bank);
	return ret;
}

static uint16_t isc_read_ccb(struct flash_bank *bank)
{
	uint8_t ccb[2];
	isc_read_register(bank, cmd_xsc_data_ccb, ccb, 16);
	return le_to_h_u16(ccb);
}

static unsigned int gucr_num(const struct flash_bank *bank)
{
	return bank->num_sectors;
}

static unsigned int sucr_num(const struct flash_bank *bank)
{
	return bank->num_sectors + 1;
}

static int isc_program_ccb(struct flash_bank *bank, uint16_t ccb)
{
	uint8_t buf[2];
	h_u16_to_le(buf, ccb);
	return isc_program_register(bank, cmd_xsc_data_ccb, buf, 16, 100);
}

static int isc_program_singe_revision_sucr(struct flash_bank *bank)
{
	uint8_t sucr[2] = {0xFC, 0xFF};
	return isc_program_register(bank, cmd_xsc_data_sucr, sucr, 16, 100);
}

static int isc_program_single_revision_btc(struct flash_bank *bank)
{
	uint8_t buf[4];
	uint32_t btc = 0xFFFFFFFF;
	btc &= ~0b1111;
	btc |= ((bank->num_sectors - 1) << 2);
	btc &= ~(1 << 4);
	h_u32_to_le(buf, btc);
	return isc_program_register(bank, cmd_xsc_data_btc, buf, 32, 100);
}

static int fpga_configure(struct flash_bank *bank)
{
	struct scan_field scan;

	scan.check_mask = NULL;
	scan.check_value = NULL;
	scan.num_bits = 16;
	scan.out_value = cmd_xsc_config;
	scan.in_value = NULL;
	jtag_add_ir_scan(bank->target->tap, &scan, TAP_IDLE);
	jtag_execute_queue();

	return ERROR_OK;
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

FLASH_BANK_COMMAND_HANDLER(xcf_flash_bank_command)
{
	struct xcf_priv *priv;

	priv = malloc(sizeof(struct xcf_priv));
	if (!priv) {
		LOG_ERROR("no memory for flash bank info");
		return ERROR_FAIL;
	}
	bank->driver_priv = priv;
	priv->probed = false;
	return ERROR_OK;
}

static int xcf_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	const struct xcf_priv *priv = bank->driver_priv;

	if (!priv->probed) {
		command_print_sameline(cmd, "\nXCF flash bank not probed yet\n");
		return ERROR_OK;
	}
	command_print_sameline(cmd, "%s", product_name(bank));
	return ERROR_OK;
}

static int xcf_probe(struct flash_bank *bank)
{
	struct xcf_priv *priv = bank->driver_priv;
	uint32_t id;

	if (priv->probed)
		free(bank->sectors);
	priv->probed = false;

	if (!bank->target->tap) {
		LOG_ERROR("Target has no JTAG tap");
		return ERROR_FAIL;
	}

	/* check idcode and alloc memory for sector table */
	if (!bank->target->tap->hasidcode)
		return ERROR_FLASH_OPERATION_FAILED;

	/* guess number of blocks using chip ID */
	id = bank->target->tap->idcode;
	switch (id & ID_MEANINGFUL_MASK) {
		case ID_XCF08P:
			bank->num_sectors = 1;
			break;
		case ID_XCF16P:
			bank->num_sectors = 2;
			break;
		case ID_XCF32P:
			bank->num_sectors = 4;
			break;
		default:
			LOG_ERROR("Unknown flash device ID 0x%" PRIX32, id);
			return ERROR_FAIL;
	}

	bank->sectors = malloc(bank->num_sectors * sizeof(struct flash_sector));
	if (!bank->sectors) {
		LOG_ERROR("No memory for sector table");
		return ERROR_FAIL;
	}
	fill_sector_table(bank);

	priv->probed = true;
	/* REVISIT: Why is unchanged bank->driver_priv rewritten by same value? */
	bank->driver_priv = priv;

	LOG_INFO("product name: %s", product_name(bank));
	LOG_INFO("device id = 0x%" PRIX32, bank->target->tap->idcode);
	LOG_INFO("flash size = %d configuration bits",
		bank->num_sectors * XCF_DATA_SECTOR_SIZE * 8);
	LOG_INFO("number of sectors = %u", bank->num_sectors);

	return ERROR_OK;
}

static int xcf_auto_probe(struct flash_bank *bank)
{
	struct xcf_priv *priv = bank->driver_priv;

	if (priv->probed)
		return ERROR_OK;
	else
		return xcf_probe(bank);
}

static int xcf_protect_check(struct flash_bank *bank)
{
	uint8_t wrpt[2];

	isc_enter(bank);
	isc_read_register(bank, cmd_xsc_data_wrpt, wrpt, 16);
	isc_leave(bank);

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = sector_state(wrpt[0], i);

	return ERROR_OK;
}

static int xcf_erase_check(struct flash_bank *bank)
{
	uint8_t blankreg;
	struct scan_field scan;

	isc_enter(bank);

	/* Do not change this code with isc_read_register() call because it needs
	 * transition to IDLE state and pause before data retrieving. */
	scan.check_mask = NULL;
	scan.check_value = NULL;
	scan.num_bits = 16;
	scan.out_value = cmd_xsc_blank_check;
	scan.in_value = NULL;
	jtag_add_ir_scan(bank->target->tap, &scan, TAP_IDLE);
	jtag_execute_queue();
	alive_sleep(500);	/* device needs at least 0.5s to self check */

	scan.num_bits = 8;
	scan.in_value = &blankreg;
	jtag_add_dr_scan(bank->target->tap, 1, &scan, TAP_IDLE);
	jtag_execute_queue();

	isc_leave(bank);

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = sector_state(blankreg, i);

	return ERROR_OK;
}

static int xcf_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	if ((first >= bank->num_sectors)
		|| (last >= bank->num_sectors)
		|| (last < first))
		return ERROR_FLASH_SECTOR_INVALID;
	else {
		isc_enter(bank);
		isc_clear_protect(bank, first, last);
		int ret = isc_erase_sectors(bank, first, last);
		isc_leave(bank);
		return ret;
	}
}

static int xcf_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	return read_write_data(bank, NULL, buffer, false, offset, count);
}

static int xcf_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
	uint32_t count)
{
	return read_write_data(bank, buffer, NULL, true, offset, count);
}

static int xcf_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	int ret;

	isc_enter(bank);
	if (set)
		ret = isc_set_protect(bank, first, last);
	else {
		/* write protection may be removed only with following erase */
		isc_clear_protect(bank, first, last);
		ret = isc_erase_sectors(bank, first, last);
	}
	isc_leave(bank);

	return ret;
}

COMMAND_HANDLER(xcf_handle_ccb_command) {

	if (!((CMD_ARGC == 1) || (CMD_ARGC == 5)))
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	uint16_t ccb = 0xFFFF;
	isc_enter(bank);
	uint16_t old_ccb = isc_read_ccb(bank);
	isc_leave(bank);

	if (CMD_ARGC == 1) {
		LOG_INFO("current CCB = 0x%X", old_ccb);
		return ERROR_OK;
	} else {
		/* skip over flash bank */
		CMD_ARGC--;
		CMD_ARGV++;
		while (CMD_ARGC) {
			if (strcmp("external", CMD_ARGV[0]) == 0)
				ccb |= (1 << 0);
			else if (strcmp("internal", CMD_ARGV[0]) == 0)
				ccb &= ~(1 << 0);
			else if (strcmp("serial", CMD_ARGV[0]) == 0)
				ccb |= (3 << 1);
			else if (strcmp("parallel", CMD_ARGV[0]) == 0)
				ccb &= ~(3 << 1);
			else if (strcmp("slave", CMD_ARGV[0]) == 0)
				ccb |= (1 << 3);
			else if (strcmp("master", CMD_ARGV[0]) == 0)
				ccb &= ~(1 << 3);
			else if (strcmp("40", CMD_ARGV[0]) == 0)
				ccb |= (3 << 4);
			else if (strcmp("20", CMD_ARGV[0]) == 0)
				ccb &= ~(1 << 5);
			else
				return ERROR_COMMAND_SYNTAX_ERROR;
			CMD_ARGC--;
			CMD_ARGV++;
		}

		isc_enter(bank);
		int sector;

		/* GUCR sector */
		sector = gucr_num(bank);
		isc_clear_protect(bank, sector, sector);
		int ret = isc_erase_sectors(bank, sector, sector);
		if (ret != ERROR_OK)
			goto EXIT;
		ret = isc_program_ccb(bank, ccb);
		if (ret != ERROR_OK)
			goto EXIT;
		ret = isc_program_single_revision_btc(bank);
		if (ret != ERROR_OK)
			goto EXIT;
		ret = isc_set_data_done(bank, sector);
		if (ret != ERROR_OK)
			goto EXIT;

		/* SUCR sector */
		sector = sucr_num(bank);
		isc_clear_protect(bank, sector, sector);
		ret = isc_erase_sectors(bank, sector, sector);
		if (ret != ERROR_OK)
			goto EXIT;
		ret = isc_program_singe_revision_sucr(bank);
		if (ret != ERROR_OK)
			goto EXIT;
		ret = isc_set_data_done(bank, sector);
		if (ret != ERROR_OK)
			goto EXIT;

EXIT:
		isc_leave(bank);
		return ret;
	}
}

COMMAND_HANDLER(xcf_handle_configure_command) {

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	return fpga_configure(bank);
}

static const struct command_registration xcf_exec_command_handlers[] = {
	{
		.name = "configure",
		.handler = xcf_handle_configure_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Initiate FPGA loading procedure."
	},
	{
		.name = "ccb",
		.handler = xcf_handle_ccb_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id [('external'|'internal') "
			"('serial'|'parallel') "
			"('slave'|'master') "
			"('40'|'20')]",
		.help = "Write CCB register with supplied options and (silently) BTC "
			"register with single revision options. Display current "
			"CCB value when only bank_id supplied. "
			"Following options available: "
			"1) external or internal clock source; "
			"2) serial or parallel bus mode; "
			"3) slave or master mode; "
			"4) clock frequency in MHz for internal clock in master mode;"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration xcf_command_handlers[] = {
	{
		.name = "xcf",
		.mode = COMMAND_ANY,
		.help = "Xilinx platform flash command group",
		.usage = "",
		.chain = xcf_exec_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver xcf_flash = {
	.name               = "xcf",
	.usage              = NULL,
	.commands           = xcf_command_handlers,
	.flash_bank_command = xcf_flash_bank_command,
	.erase              = xcf_erase,
	.protect            = xcf_protect,
	.write              = xcf_write,
	.read               = xcf_read,
	.probe              = xcf_probe,
	.auto_probe         = xcf_auto_probe,
	.erase_check        = xcf_erase_check,
	.protect_check      = xcf_protect_check,
	.info               = xcf_info,
	.free_driver_priv   = default_flash_free_driver_priv,
};
