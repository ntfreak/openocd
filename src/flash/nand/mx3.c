
/***************************************************************************
 *   Copyright (C) 2009 by Alexei Babich                                   *
 *   Rezonans plc., Chelyabinsk, Russia                                    *
 *   impatt@mail.ru                                                        *
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

/*
 * Freescale iMX3* OpenOCD NAND Flash controller support.
 *
 * Many thanks to Ben Dooks for writing s3c24xx driver.
 */

/*
driver tested with STMicro NAND512W3A @imx31
tested "nand probe #", "nand erase # 0 #", "nand dump # file 0 #", "nand write # file 0"
get_next_halfword_from_sram_buffer() not tested
*/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "mx3.h"
#include <target/target.h>

static const char target_not_halted_err_msg[] =
		"target must be halted to use mx3 NAND flash controller";
static const char data_block_size_err_msg[] =
		"minimal granularity is one half-word, %" PRIu32 " is incorrect";
static const char sram_buffer_bounds_err_msg[] =
		"trying to access out of SRAM buffer bound (addr=0x%" PRIx32 ")";
static const char get_status_register_err_msg[] = "can't get NAND status";
static uint32_t in_sram_address;
static unsigned char sign_of_sequental_byte_read;

static int test_iomux_settings(struct target *target, uint32_t value,
		uint32_t mask, const char *text);
static int initialize_nf_controller(struct nand_device *nand);
static int get_next_byte_from_sram_buffer(struct target *target, uint8_t *value);
static int get_next_halfword_from_sram_buffer(struct target *target,
		uint16_t *value);
static int poll_for_complete_op(struct target *target, const char *text);
static int validate_target_state(struct nand_device *nand);
static int do_data_output(struct nand_device *nand);

static int imx31_command(struct nand_device *nand, uint8_t command);
static int imx31_address(struct nand_device *nand, uint8_t address);

NAND_DEVICE_COMMAND_HANDLER(imx31_nand_device_command)
{
	struct mx3_nf_controller *mx3_nf_info;
	mx3_nf_info = malloc(sizeof(struct mx3_nf_controller));
	if (mx3_nf_info == NULL) {
		LOG_ERROR("no memory for nand controller");
		return ERROR_FAIL;
	}

	nand->controller_priv = mx3_nf_info;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;
	/*
	* check hwecc requirements
	*/
	{
		int hwecc_needed;
		hwecc_needed = strcmp(CMD_ARGV[2], "hwecc");
		if (hwecc_needed == 0)
			mx3_nf_info->flags.hw_ecc_enabled = 1;
		else
			mx3_nf_info->flags.hw_ecc_enabled = 0;
	}

	mx3_nf_info->optype = MX3_NF_DATAOUT_PAGE;
	mx3_nf_info->fin = MX3_NF_FIN_NONE;
	mx3_nf_info->flags.target_little_endian =
			(nand->target->endianness == TARGET_LITTLE_ENDIAN);

	return ERROR_OK;
}

static int imx31_init(struct nand_device *nand)
{
	struct mx3_nf_controller *mx3_nf_info = nand->controller_priv;
	struct target *target = nand->target;

	{
		/*
		 * validate target state
		 */
		int validate_target_result;
		validate_target_result = validate_target_state(nand);
		if (validate_target_result != ERROR_OK)
			return validate_target_result;
	}

	{
		uint16_t buffsize_register_content;
		target_read_u16(target, MX3_NF_BUFSIZ, &buffsize_register_content);
		mx3_nf_info->flags.one_kb_sram = !(buffsize_register_content & 0x000f);
	}

	{
		uint32_t pcsr_register_content;
		target_read_u32(target, MX3_PCSR, &pcsr_register_content);
		if (!nand->bus_width) {
			nand->bus_width = (pcsr_register_content & 0x80000000) ? 16 : 8;
		} else {
			pcsr_register_content |= ((nand->bus_width == 16) ? 0x80000000 : 0x00000000);
			target_write_u32(target, MX3_PCSR, pcsr_register_content);
		}

		if (!nand->page_size) {
			nand->page_size = (pcsr_register_content & 0x40000000) ? 2048 : 512;
		} else {
			pcsr_register_content |= ((nand->page_size == 2048) ? 0x40000000 : 0x00000000);
			target_write_u32(target, MX3_PCSR, pcsr_register_content);
		}
		if (mx3_nf_info->flags.one_kb_sram && (nand->page_size == 2048)) {
			LOG_ERROR("NAND controller have only 1 kb SRAM, "
					"so pagesize 2048 is incompatible with it");
		}
	}

	{
		uint32_t cgr_register_content;
		target_read_u32(target, MX3_CCM_CGR2, &cgr_register_content);
		if (!(cgr_register_content & 0x00000300)) {
			LOG_ERROR("clock gating to EMI disabled");
			return ERROR_FAIL;
		}
	}

	{
		uint32_t gpr_register_content;
		target_read_u32(target, MX3_GPR, &gpr_register_content);
		if (gpr_register_content & 0x00000060) {
			LOG_ERROR("pins mode overridden by GPR");
			return ERROR_FAIL;
		}
	}

	{
		/*
		 * testing IOMUX settings; must be in "functional-mode output and
		 * functional-mode input" mode
		 */
		int test_iomux;
		test_iomux = ERROR_OK;
		test_iomux |= test_iomux_settings(target, 0x43fac0c0, 0x7f7f7f00, "d0,d1,d2");
		test_iomux |= test_iomux_settings(target, 0x43fac0c4, 0x7f7f7f7f, "d3,d4,d5,d6");
		test_iomux |= test_iomux_settings(target, 0x43fac0c8, 0x0000007f, "d7");
		if (nand->bus_width == 16) {
			test_iomux |= test_iomux_settings(target, 0x43fac0c8, 0x7f7f7f00, "d8,d9,d10");
			test_iomux |= test_iomux_settings(target, 0x43fac0cc, 0x7f7f7f7f, "d11,d12,d13,d14");
			test_iomux |= test_iomux_settings(target, 0x43fac0d0, 0x0000007f, "d15");
		}
		test_iomux |= test_iomux_settings(target, 0x43fac0d0, 0x7f7f7f00, "nfwp,nfce,nfrb");
		test_iomux |= test_iomux_settings(target, 0x43fac0d4, 0x7f7f7f7f,
				"nfwe,nfre,nfale,nfcle");
		if (test_iomux != ERROR_OK)
			return ERROR_FAIL;
	}

	initialize_nf_controller(nand);

	{
		int retval;
		uint16_t nand_status_content;
		retval = ERROR_OK;
		retval |= imx31_command(nand, NAND_CMD_STATUS);
		retval |= imx31_address(nand, 0x00);
		retval |= do_data_output(nand);
		if (retval != ERROR_OK) {
			LOG_ERROR(get_status_register_err_msg);
			return ERROR_FAIL;
		}
		target_read_u16(target, MX3_NF_MAIN_BUFFER0, &nand_status_content);
		if (!(nand_status_content & 0x0080)) {
			/*
			 * is host-big-endian correctly ??
			 */
			LOG_INFO("NAND read-only");
			mx3_nf_info->flags.nand_readonly = 1;
		} else
			mx3_nf_info->flags.nand_readonly = 0;
	}
	return ERROR_OK;
}

static int imx31_read_data(struct nand_device *nand, void *data)
{
	struct target *target = nand->target;
	{
		/*
		 * validate target state
		 */
		int validate_target_result;
		validate_target_result = validate_target_state(nand);
		if (validate_target_result != ERROR_OK)
			return validate_target_result;
	}

	{
		/*
		 * get data from nand chip
		 */
		int try_data_output_from_nand_chip;
		try_data_output_from_nand_chip = do_data_output(nand);
		if (try_data_output_from_nand_chip != ERROR_OK)
			return try_data_output_from_nand_chip;
	}

	if (nand->bus_width == 16)
		get_next_halfword_from_sram_buffer(target, data);
	else
		get_next_byte_from_sram_buffer(target, data);

	return ERROR_OK;
}

static int imx31_write_data(struct nand_device *nand, uint16_t data)
{
	LOG_ERROR("write_data() not implemented");
	return ERROR_NAND_OPERATION_FAILED;
}

static int imx31_reset(struct nand_device *nand)
{
	/*
	* validate target state
	*/
	int validate_target_result;
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;
	initialize_nf_controller(nand);
	return ERROR_OK;
}

static int imx31_command(struct nand_device *nand, uint8_t command)
{
	struct mx3_nf_controller *mx3_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	{
		/*
		 * validate target state
		 */
		int validate_target_result;
		validate_target_result = validate_target_state(nand);
		if (validate_target_result != ERROR_OK)
			return validate_target_result;
	}

	switch (command) {
		case NAND_CMD_READOOB:
			command = NAND_CMD_READ0;
			in_sram_address = MX3_NF_SPARE_BUFFER0;	/* set read point for
								* data_read() and
								* read_block_data() to
								* spare area in SRAM
								* buffer */
			break;
		case NAND_CMD_READ1:
			command = NAND_CMD_READ0;
			/*
			 * offset == one half of page size
			 */
			in_sram_address = MX3_NF_MAIN_BUFFER0 + (nand->page_size >> 1);
			break;
		default:
			in_sram_address = MX3_NF_MAIN_BUFFER0;
	}

	target_write_u16(target, MX3_NF_FCMD, command);
	/*
	* start command input operation (set MX3_NF_BIT_OP_DONE==0)
	*/
	target_write_u16(target, MX3_NF_CFG2, MX3_NF_BIT_OP_FCI);
	{
		int poll_result;
		poll_result = poll_for_complete_op(target, "command");
		if (poll_result != ERROR_OK)
			return poll_result;
	}
	/*
	* reset cursor to begin of the buffer
	*/
	sign_of_sequental_byte_read = 0;
	switch (command) {
		case NAND_CMD_READID:
			mx3_nf_info->optype = MX3_NF_DATAOUT_NANDID;
			mx3_nf_info->fin = MX3_NF_FIN_DATAOUT;
			break;
		case NAND_CMD_STATUS:
			mx3_nf_info->optype = MX3_NF_DATAOUT_NANDSTATUS;
			mx3_nf_info->fin = MX3_NF_FIN_DATAOUT;
			break;
		case NAND_CMD_READ0:
			mx3_nf_info->fin = MX3_NF_FIN_DATAOUT;
			mx3_nf_info->optype = MX3_NF_DATAOUT_PAGE;
			break;
		default:
			mx3_nf_info->optype = MX3_NF_DATAOUT_PAGE;
	}
	return ERROR_OK;
}

static int imx31_address(struct nand_device *nand, uint8_t address)
{
	struct target *target = nand->target;
	{
		/*
		 * validate target state
		 */
		int validate_target_result;
		validate_target_result = validate_target_state(nand);
		if (validate_target_result != ERROR_OK)
			return validate_target_result;
	}

	target_write_u16(target, MX3_NF_FADDR, address);
	/*
	* start address input operation (set MX3_NF_BIT_OP_DONE==0)
	*/
	target_write_u16(target, MX3_NF_CFG2, MX3_NF_BIT_OP_FAI);
	{
		int poll_result;
		poll_result = poll_for_complete_op(target, "address");
		if (poll_result != ERROR_OK)
			return poll_result;
	}
	return ERROR_OK;
}

static int imx31_nand_ready(struct nand_device *nand, int tout)
{
	uint16_t poll_complete_status;
	struct target *target = nand->target;

	{
		/*
		 * validate target state
		 */
		int validate_target_result;
		validate_target_result = validate_target_state(nand);
		if (validate_target_result != ERROR_OK)
			return validate_target_result;
	}

	do {
		target_read_u16(target, MX3_NF_CFG2, &poll_complete_status);
		if (poll_complete_status & MX3_NF_BIT_OP_DONE)
			return tout;
		alive_sleep(1);
	} while (tout-- > 0);
	return tout;
}

static int imx31_write_page(struct nand_device *nand, uint32_t page,
		uint8_t *data, uint32_t data_size, uint8_t *oob,
		uint32_t oob_size)
{
	struct mx3_nf_controller *mx3_nf_info = nand->controller_priv;
	struct target *target = nand->target;

	if (data_size % 2) {
		LOG_ERROR(data_block_size_err_msg, data_size);
		return ERROR_NAND_OPERATION_FAILED;
	}
	if (oob_size % 2) {
		LOG_ERROR(data_block_size_err_msg, oob_size);
		return ERROR_NAND_OPERATION_FAILED;
	}
	if (!data) {
		LOG_ERROR("nothing to program");
		return ERROR_NAND_OPERATION_FAILED;
	}
	{
		/*
		 * validate target state
		 */
		int retval;
		retval = validate_target_state(nand);
		if (retval != ERROR_OK)
			return retval;
	}
	{
		int retval = ERROR_OK;
		retval |= imx31_command(nand, NAND_CMD_SEQIN);
		retval |= imx31_address(nand, 0x00);
		retval |= imx31_address(nand, page & 0xff);
		retval |= imx31_address(nand, (page >> 8) & 0xff);
		if (nand->address_cycles >= 4) {
			retval |= imx31_address(nand, (page >> 16) & 0xff);
			if (nand->address_cycles >= 5)
				retval |= imx31_address(nand, (page >> 24) & 0xff);
		}
		target_write_buffer(target, MX3_NF_MAIN_BUFFER0, data_size, data);
		if (oob) {
			if (mx3_nf_info->flags.hw_ecc_enabled) {
				/*
				 * part of spare block will be overridden by hardware
				 * ECC generator
				 */
				LOG_DEBUG("part of spare block will be overridden by hardware ECC generator");
			}
			target_write_buffer(target, MX3_NF_SPARE_BUFFER0, oob_size, oob);
		}
		/*
		 * start data input operation (set MX3_NF_BIT_OP_DONE==0)
		 */
		target_write_u16(target, MX3_NF_CFG2, MX3_NF_BIT_OP_FDI);
		{
			int poll_result;
			poll_result = poll_for_complete_op(target, "data input");
			if (poll_result != ERROR_OK)
				return poll_result;
		}
		retval |= imx31_command(nand, NAND_CMD_PAGEPROG);
		if (retval != ERROR_OK)
			return retval;

		/*
		 * check status register
		 */
		{
			uint16_t nand_status_content;
			retval = ERROR_OK;
			retval |= imx31_command(nand, NAND_CMD_STATUS);
			retval |= imx31_address(nand, 0x00);
			retval |= do_data_output(nand);
			if (retval != ERROR_OK) {
				LOG_ERROR(get_status_register_err_msg);
				return retval;
			}
			target_read_u16(target, MX3_NF_MAIN_BUFFER0, &nand_status_content);
			if (nand_status_content & 0x0001) {
				/*
				 * is host-big-endian correctly ??
				 */
				return ERROR_NAND_OPERATION_FAILED;
			}
		}
	}
	return ERROR_OK;
}

static int imx31_read_page(struct nand_device *nand, uint32_t page,
		uint8_t *data, uint32_t data_size, uint8_t *oob,
		uint32_t oob_size)
{
	struct target *target = nand->target;

	if (data_size % 2) {
		LOG_ERROR(data_block_size_err_msg, data_size);
		return ERROR_NAND_OPERATION_FAILED;
	}
	if (oob_size % 2) {
		LOG_ERROR(data_block_size_err_msg, oob_size);
		return ERROR_NAND_OPERATION_FAILED;
	}

	{
		/*
		 * validate target state
		 */
		int retval;
		retval = validate_target_state(nand);
		if (retval != ERROR_OK)
			return retval;
	}
	{
		int retval = ERROR_OK;
		retval |= imx31_command(nand, NAND_CMD_READ0);
		retval |= imx31_address(nand, 0x00);
		retval |= imx31_address(nand, page & 0xff);
		retval |= imx31_address(nand, (page >> 8) & 0xff);
		if (nand->address_cycles >= 4) {
			retval |= imx31_address(nand, (page >> 16) & 0xff);
			if (nand->address_cycles >= 5) {
				retval |= imx31_address(nand, (page >> 24) & 0xff);
				retval |= imx31_command(nand, NAND_CMD_READSTART);
			}
		}
		retval |= do_data_output(nand);
		if (retval != ERROR_OK)
			return retval;

		if (data) {
			target_read_buffer(target, MX3_NF_MAIN_BUFFER0, data_size,
				data);
		}
		if (oob) {
			target_read_buffer(target, MX3_NF_SPARE_BUFFER0, oob_size,
				oob);
		}
	}
	return ERROR_OK;
}

static int test_iomux_settings(struct target *target, uint32_t address,
		uint32_t mask, const char *text)
{
	uint32_t register_content;
	target_read_u32(target, address, &register_content);
	if ((register_content & mask) != (0x12121212 & mask)) {
		LOG_ERROR("IOMUX for {%s} is bad", text);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int initialize_nf_controller(struct nand_device *nand)
{
	struct mx3_nf_controller *mx3_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	/*
	* resets NAND flash controller in zero time ? I don't know.
	*/
	target_write_u16(target, MX3_NF_CFG1, MX3_NF_BIT_RESET_EN);
	{
		uint16_t work_mode;
		work_mode = MX3_NF_BIT_INT_DIS;	/* disable interrupt */
		if (target->endianness == TARGET_BIG_ENDIAN)
			work_mode |= MX3_NF_BIT_BE_EN;
		if (mx3_nf_info->flags.hw_ecc_enabled)
			work_mode |= MX3_NF_BIT_ECC_EN;
		target_write_u16(target, MX3_NF_CFG1, work_mode);
	}
	/*
	* unlock SRAM buffer for write; 2 mean "Unlock", other values means "Lock"
	*/
	target_write_u16(target, MX3_NF_BUFCFG, 2);
	{
		uint16_t temp;
		target_read_u16(target, MX3_NF_FWP, &temp);
		if ((temp & 0x0007) == 1) {
			LOG_ERROR("NAND flash is tight-locked, reset needed");
			return ERROR_FAIL;
		}

	}
	/*
	* unlock NAND flash for write
	*/
	target_write_u16(target, MX3_NF_FWP, 4);
	target_write_u16(target, MX3_NF_LOCKSTART, 0x0000);
	target_write_u16(target, MX3_NF_LOCKEND, 0xFFFF);
	/*
	* 0x0000 means that first SRAM buffer @0xB800_0000 will be used
	*/
	target_write_u16(target, MX3_NF_BUFADDR, 0x0000);
	/*
	* address of SRAM buffer
	*/
	in_sram_address = MX3_NF_MAIN_BUFFER0;
	sign_of_sequental_byte_read = 0;
	return ERROR_OK;
}

static int get_next_byte_from_sram_buffer(struct target *target, uint8_t *value)
{
	static uint8_t even_byte;
	/*
	* host-big_endian ??
	*/
	if (sign_of_sequental_byte_read == 0)
		even_byte = 0;
	if (in_sram_address > MX3_NF_LAST_BUFFER_ADDR) {
		LOG_ERROR(sram_buffer_bounds_err_msg, in_sram_address);
		*value = 0;
		sign_of_sequental_byte_read = 0;
		even_byte = 0;
		return ERROR_NAND_OPERATION_FAILED;
	} else {
		uint16_t temp;
		target_read_u16(target, in_sram_address, &temp);
		if (even_byte) {
			*value = temp >> 8;
			even_byte = 0;
			in_sram_address += 2;
		} else {
			*value = temp & 0xff;
			even_byte = 1;
		}
	}
	sign_of_sequental_byte_read = 1;
	return ERROR_OK;
}

static int get_next_halfword_from_sram_buffer(struct target *target,
		uint16_t *value)
{
	if (in_sram_address > MX3_NF_LAST_BUFFER_ADDR) {
		LOG_ERROR(sram_buffer_bounds_err_msg, in_sram_address);
		*value = 0;
		return ERROR_NAND_OPERATION_FAILED;
	} else {
		target_read_u16(target, in_sram_address, value);
		in_sram_address += 2;
	}
	return ERROR_OK;
}

static int poll_for_complete_op(struct target *target, const char *text)
{
	uint16_t poll_complete_status;
	for (int poll_cycle_count = 0; poll_cycle_count < 100; poll_cycle_count++) {
		usleep(25);
		target_read_u16(target, MX3_NF_CFG2, &poll_complete_status);
		if (poll_complete_status & MX3_NF_BIT_OP_DONE)
			break;
	}
	if (!(poll_complete_status & MX3_NF_BIT_OP_DONE)) {
		LOG_ERROR("%s sending timeout", text);
		return ERROR_NAND_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int validate_target_state(struct nand_device *nand)
{
	struct mx3_nf_controller *mx3_nf_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR(target_not_halted_err_msg);
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (mx3_nf_info->flags.target_little_endian !=
			(target->endianness == TARGET_LITTLE_ENDIAN)) {
		/*
		 * endianness changed after NAND controller probed
		 */
		return ERROR_NAND_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int do_data_output(struct nand_device *nand)
{
	struct mx3_nf_controller *mx3_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	switch (mx3_nf_info->fin) {
		case MX3_NF_FIN_DATAOUT:
			/*
			 * start data output operation (set MX3_NF_BIT_OP_DONE==0)
			 */
			target_write_u16 (target, MX3_NF_CFG2,
					MX3_NF_BIT_DATAOUT_TYPE(mx3_nf_info->optype));
			{
				int poll_result;
				poll_result = poll_for_complete_op(target, "data output");
				if (poll_result != ERROR_OK)
					return poll_result;
			}
			mx3_nf_info->fin = MX3_NF_FIN_NONE;
			/*
			 * ECC stuff
			 */
			if ((mx3_nf_info->optype == MX3_NF_DATAOUT_PAGE)
					&& mx3_nf_info->flags.hw_ecc_enabled) {
				uint16_t ecc_status;
				target_read_u16 (target, MX3_NF_ECCSTATUS, &ecc_status);
				switch (ecc_status & 0x000c) {
				case 1 << 2:
					LOG_DEBUG("main area read with 1 (correctable) error");
					break;
				case 2 << 2:
					LOG_DEBUG("main area read with more than 1 (incorrectable) error");
					return ERROR_NAND_OPERATION_FAILED;
				}
				switch (ecc_status & 0x0003) {
				case 1:
					LOG_DEBUG("spare area read with 1 (correctable) error");
					break;
				case 2:
					LOG_DEBUG("main area read with more than 1 (incorrectable) error");
					return ERROR_NAND_OPERATION_FAILED;
				}
			}
			break;
		case MX3_NF_FIN_NONE:
			break;
	}
	return ERROR_OK;
}

struct nand_flash_controller imx31_nand_flash_controller = {
	.name = "imx31",
	.usage = "nand device imx31 target noecc|hwecc",
	.nand_device_command = &imx31_nand_device_command,
	.init = &imx31_init,
	.reset = &imx31_reset,
	.command = &imx31_command,
	.address = &imx31_address,
	.write_data = &imx31_write_data,
	.read_data = &imx31_read_data,
	.write_page = &imx31_write_page,
	.read_page = &imx31_read_page,
	.nand_ready = &imx31_nand_ready,
};
