/***************************************************************************
 *   Copyright (C) 2019 by UNISOC                                          *
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

#include <helper/types.h>
#include "uwp5661.h"
#include "spi.h"

static int uwp5661_init_sw_csn(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t rd_dat = 0;
	/* GPIO init for faster programming */
	/* use CSN as GPIO bit28: GROUP1 BIT12 */
	/* set GPIO_MODE1 register to select GPIO's group. */
	retval = target_read_u32(target, REG_AON_GLB_RF_GPIO_MODE1, &rd_dat);
	if (retval != ERROR_OK)
		return retval;
	rd_dat &= (~(BIT(28)));
	retval = target_write_u32(target, REG_AON_GLB_RF_GPIO_MODE1, rd_dat);
	if (retval != ERROR_OK)
		return retval;
	/* GPIO1_EB */
	retval = target_write_u32(target, REG_AON_GLB_RF_APB_EB_SET, BIT(13));
	if (retval != ERROR_OK)
		return retval;
	/* GPIO bit28 Dir */
	retval = target_read_u32(target, REG_AON_GPIO1_RF_GPIO_DIR, &rd_dat);
	if (retval != ERROR_OK)
		return retval;
	rd_dat |= BIT(12);
	retval = target_write_u32(target, REG_AON_GPIO1_RF_GPIO_DIR, rd_dat);
	if (retval != ERROR_OK)
		return retval;
	/* GPIO bit28 Mask */
	retval = target_read_u32(target, REG_AON_GPIO1_RF_GPIO_MSK, &rd_dat);
	if (retval != ERROR_OK)
		return retval;
	rd_dat |= BIT(12);
	retval = target_write_u32(target, REG_AON_GPIO1_RF_GPIO_MSK, rd_dat);
	if (retval != ERROR_OK)
		return retval;
	/* GPIO bit28 Value */
	retval = target_read_u32(target, REG_AON_GPIO1_RF_GPIO_VAL, &rd_dat);
	if (retval != ERROR_OK)
		return retval;
	rd_dat &= (~BIT(12));
	retval = target_write_u32(target, REG_AON_GPIO1_RF_GPIO_VAL, rd_dat);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int uwp5661_force_csn(struct target *target, uint32_t op)
{
	int retval = ERROR_OK;
	uint32_t rd_dat;
	if (op == true) {  /* use CS as GPIO */
		retval = target_read_u32(target, REG_AON_PIN_RF_ESMCSN_CFG, &rd_dat);
		if (retval != ERROR_OK)
			return retval;
		rd_dat &= (~FUNC_MSK);
		rd_dat |= FUNC_3;
		retval = target_write_u32(target, REG_AON_PIN_RF_ESMCSN_CFG, rd_dat);
		if (retval != ERROR_OK)
			return retval;
	} else {  /* not GPIO */
		retval = target_read_u32(target, REG_AON_PIN_RF_ESMCSN_CFG, &rd_dat);
		if (retval != ERROR_OK)
			return retval;
		rd_dat &= (~FUNC_MSK);
		retval = target_write_u32(target, REG_AON_PIN_RF_ESMCSN_CFG, rd_dat);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int uwp5661_set_sfc_clk(struct target *target)
{
	int retval = ERROR_OK;
	retval = target_write_u32(target, SFC_CLK_CFG, SFC_CLK_OUT_2X_EN |
			SFC_CLK_2X_EN | SFC_CLK_SAMPLE_2X_PHASE_1 | SFC_CLK_SAMPLE_2X_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, REG_AON_CLK_RF_CGM_SFC_1X_CFG, CLK_SFC_1X_DIV_2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, REG_AON_CLK_RF_CGM_SFC_2X_CFG, CLK_SFC_2X_XTAL_MHZ);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int sfcdrv_req(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t int_status = 0;
	uint32_t int_timeout = 0;
	retval = target_write_u32(target, SFC_SOFT_REQ, (1 << SHIFT_SOFT_REQ));
	if (retval != ERROR_OK)
		return retval;
	do {
		retval = target_read_u32(target, SFC_INT_RAW, &int_status);
		if (retval != ERROR_OK)
			return retval;
		if (int_timeout++ > SFC_DRVREQ_TIMEOUT) {
			LOG_ERROR("SFCDRV Req time out!\n");
			break;
		}
	} while (int_status == 0);
	retval = target_write_u32(target, SFC_INT_CLR , (1 << SHIFT_INT_CLR));
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int sfcdrv_int_cfg(struct target *target, uint32_t op)
{
	int retval = ERROR_OK;
	if (op == true) {  /* for CS1 interrupt */
		retval = target_write_u32(target, SFC_IEN, CS1_INT);
		if (retval != ERROR_OK)
			return retval;
	} else { /* for CS0 interrupt */
		retval = target_write_u32(target, SFC_IEN, CS0_INT);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static uint32_t sfcdrv_get_init_addr(struct uwp5661_flash_bank *uwp5661_info, struct target *target)
{
	int retval = ERROR_OK;
	uint32_t start_addr = uwp5661_info->sfc_cmd_cfg_cache ;

	if (uwp5661_info->sfc_cmd_cfg_cache == 0xFFFFFFFF) {	/* if cfg_cache is initial value */
		retval = target_read_u32(target, SFC_CMD_CFG, &start_addr);
		if (retval != ERROR_OK)
			LOG_ERROR("read start addr failed.");
	}

	start_addr = (start_addr & MSK_STS_INI_ADDR_SEL) >> SHIFT_STS_INI_ADDR_SEL;

	switch (start_addr) {
		case INI_CMD_BUF_6:
			start_addr = CMD_BUF_6;
			break;

		case INI_CMD_BUF_5:
			start_addr = CMD_BUF_5;
			break;

		case INI_CMD_BUF_4:
			start_addr = CMD_BUF_4;
			break;

		default:
			start_addr = CMD_BUF_7;
			break;
	}
	return start_addr;
}

static int sfcdrv_set_cmd_cfg_reg(struct uwp5661_flash_bank *uwp5661_info, struct target *target,
				enum cmd_mode cmdmode, enum bit_mode bitmode, enum ini_add_sel iniaddsel)
{
	int retval = ERROR_OK;
	uint32_t nxt_sfc_cmd_cfg = ((cmdmode << SHIFT_CMD_SET)|
								(bitmode << SHIFT_RDATA_BIT_MODE)|
								(iniaddsel << SHIFT_STS_INI_ADDR_SEL));

	if (uwp5661_info->sfc_cmd_cfg_cache != nxt_sfc_cmd_cfg) {
		retval = target_write_u32(target, SFC_CMD_CFG, nxt_sfc_cmd_cfg);
		if (retval != ERROR_OK)
			return retval;
		uwp5661_info->sfc_cmd_cfg_cache = nxt_sfc_cmd_cfg;
	}

	return retval;
}

static void sfcdrv_set_cmd_buf(struct uwp5661_flash_bank *uwp5661_info,	enum cmd_buf_index index, uint32_t value)
{
	uwp5661_info->cmd_buf_cache_bitmap |= 1<<index;
	uwp5661_info->cmd_info_buf_cache[index] = value;
}

static void sfcdrv_set_type_inf_buf(struct uwp5661_flash_bank *uwp5661_info,
						enum cmd_buf_index index, struct sfc_cmd_des *cmd_des)
{
	uwp5661_info->cmd_info_buf_cache[INFO_BUF_0 + (index>>2)] |= ((VALID0|
												(cmd_des->bit_mode << SHIFT_BIT_MODE0)|
												(cmd_des->cmd_byte_len << SHIFT_BYTE_NUM0)|
												(cmd_des->cmd_mode << SHIFT_OPERATION_STATUS0)|
												(cmd_des->send_mode << SHIFT_BYTE_SEND_MODE0)))<<(8*(index%4));
}

static int sfcdrv_get_read_buf(struct uwp5661_flash_bank *uwp5661_info, struct target *target,
					uint32_t *buffer, uint32_t word_cnt)
{
	int retval = ERROR_OK;
	uint32_t i = 0;
	uint32_t read_buf_index = sfcdrv_get_init_addr(uwp5661_info, target);
	uint8_t tmp_buf[INFO_BUF_MAX*4] = {0};

	retval = target_read_memory(target, SFC_CMD_BUF0+read_buf_index*4, 4, word_cnt, tmp_buf);
	if (retval != ERROR_OK)
		return retval;
	for (i = 0; i < word_cnt; i++)
		buffer[i] = target_buffer_get_u32(target, tmp_buf+i*4);

	return retval;
}

static int sfcdrv_set_cmd_data(struct uwp5661_flash_bank *uwp5661_info,
					uint32_t cmd_buf_index, struct sfc_cmd_des *cmd_des)
{
	if (!cmd_des)
		return ERROR_FAIL;
	sfcdrv_set_cmd_buf(uwp5661_info, cmd_buf_index, cmd_des->cmd);
	sfcdrv_set_type_inf_buf(uwp5661_info, cmd_buf_index, cmd_des);
	return ERROR_OK;
}

static int sfcdrv_set_read_buf(struct uwp5661_flash_bank *uwp5661_info,
					uint32_t read_buf_index, struct sfc_cmd_des *cmd_des)
{
	if (!cmd_des)
		return ERROR_FAIL;
	sfcdrv_set_type_inf_buf(uwp5661_info, read_buf_index, cmd_des);
	return ERROR_OK;
}

static void create_cmd(struct sfc_cmd_des *cmd_desc, uint32_t cmd, uint32_t byte_len,
			enum cmd_mode cmd_mode, enum bit_mode bit_mode, enum send_mode send_mode)
{
	cmd_desc->cmd = cmd;
	cmd_desc->cmd_byte_len = byte_len;
	cmd_desc->cmd_mode = cmd_mode;
	cmd_desc->bit_mode = bit_mode;
	cmd_desc->send_mode = send_mode;
}

static int uwp5661_read_write(struct uwp5661_flash_bank *uwp5661_info, struct target *target,
			struct sfc_cmd_des *cmd_des, uint32_t cmd_len, uint32_t *din)
{
	int retval = ERROR_OK;
	uint32_t i = 0;
	uint32_t read_count = 0;
	uint32_t read_buf_index = sfcdrv_get_init_addr(uwp5661_info, target);
	uint8_t tmp_buf[INFO_BUF_MAX*4] = {0};
	uint32_t update_info_buf = false;

	uwp5661_info->cmd_buf_cache_bitmap = 0;
	memset(uwp5661_info->cmd_info_buf_cache, 0 , sizeof(uwp5661_info->cmd_info_buf_cache));

	for (i = 0; i < cmd_len; i++) {
		if ((cmd_des[i].cmd_mode == CMD_MODE_WRITE) || (cmd_des[i].cmd_mode == CMD_MODE_HIGHZ)) {
			retval = sfcdrv_set_cmd_data(uwp5661_info, i, &(cmd_des[i]));
			if (retval != ERROR_OK)
				return retval;
		} else if (cmd_des[i].cmd_mode == CMD_MODE_READ) {
			sfcdrv_set_cmd_buf(uwp5661_info, read_buf_index, 0);
			retval = sfcdrv_set_read_buf(uwp5661_info, read_buf_index, &(cmd_des[i]));
			if (retval != ERROR_OK)
				return retval;
			read_buf_index++;
			read_count++;
		}
	}

	if ((uwp5661_info->prev_cmd_info_buf_cache[INFO_BUF_0] != uwp5661_info->cmd_info_buf_cache[INFO_BUF_0]) ||
		(uwp5661_info->prev_cmd_info_buf_cache[INFO_BUF_1] != uwp5661_info->cmd_info_buf_cache[INFO_BUF_1]) ||
		(uwp5661_info->prev_cmd_info_buf_cache[INFO_BUF_2] != uwp5661_info->cmd_info_buf_cache[INFO_BUF_2])) {
		for (i = INFO_BUF_0; i < INFO_BUF_MAX; i++) {
			target_buffer_set_u32(target, tmp_buf+i*4, uwp5661_info->cmd_info_buf_cache[i]);
			uwp5661_info->prev_cmd_info_buf_cache[i] = uwp5661_info->cmd_info_buf_cache[i];
		}

		update_info_buf = true;
	}

	if (cmd_len <= 2) {
		for (i = CMD_BUF_0; i < CMD_BUF_MAX; i++) {
			if (uwp5661_info->cmd_buf_cache_bitmap & (1<<i)) {
				retval = target_write_u32(target, SFC_CMD_BUF0+i*4, uwp5661_info->cmd_info_buf_cache[i]);
				if (retval != ERROR_OK)
					return retval;
			}
		}

		if (update_info_buf == true) {
			retval = target_write_memory(target, SFC_TYPE_BUF0, 4, INFO_BUF_MAX - INFO_BUF_0, tmp_buf+INFO_BUF_0*4);
			if (retval != ERROR_OK)
				return retval;
		}
	} else {
		if (update_info_buf == true) {
			for (i = CMD_BUF_0; i < INFO_BUF_MAX; i++)
				target_buffer_set_u32(target, tmp_buf+i*4, uwp5661_info->cmd_info_buf_cache[i]);
			retval = target_write_memory(target, SFC_CMD_BUF0, 4, INFO_BUF_MAX, tmp_buf);
			if (retval != ERROR_OK)
				return retval;
		} else {
			for (i = CMD_BUF_0; i < CMD_BUF_MAX; i++)
				target_buffer_set_u32(target, tmp_buf+i*4, uwp5661_info->cmd_info_buf_cache[i]);
			retval = target_write_memory(target, SFC_CMD_BUF0, 4, CMD_BUF_MAX, tmp_buf);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	retval = sfcdrv_req(target);
	if (retval != ERROR_OK)
		return retval;

	if (0 != read_count) {
		retval = sfcdrv_get_read_buf(uwp5661_info, target, din, read_count);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int uwp5661_disable_cache(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t int_status = 0;
	uint32_t int_timeout = 0;

	retval = target_write_u32(target, REG_ICACHE_INT_EN, RF_CMD_IRQ_DISABLE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_ICACHE_INT_CLR, RF_CMD_IRQ_CLR);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_ICACHE_CMD_CFG2, RF_CMD_STR_CMD_START|RF_CMD_TYPE_INVALID_ALL);
	if (retval != ERROR_OK)
		return retval;
	do {
		retval = target_read_u32(target, REG_ICACHE_INT_RAW_STS, &int_status);
		if (retval != ERROR_OK)
			return retval;
		if (int_timeout++ > CACHE_CMD_TIMEOUT) {
			LOG_ERROR("ICache invalid time out!\n");
			break;
		}
	} while ((int_status & BIT(0)) == 0);
	retval = target_write_u32(target, REG_ICACHE_INT_CLR, RF_CMD_IRQ_CLR);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_ICACHE_CFG0, ALL_DISABLED);
	if (retval != ERROR_OK)
		return retval;

	int_timeout = 0;
	retval = target_write_u32(target, REG_DCACHE_INT_EN, RF_CMD_IRQ_DISABLE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_DCACHE_INT_CLR, RF_CMD_IRQ_CLR);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_DCACHE_CMD_CFG2, RF_CMD_STR_CMD_START|RF_CMD_TYPE_CLEAN_INVALID_ALL);
	if (retval != ERROR_OK)
		return retval;
	do {
		retval = target_read_u32(target, REG_DCACHE_INT_RAW_STS, &int_status);
		if (retval != ERROR_OK)
			return retval;
		if (int_timeout++ > CACHE_CMD_TIMEOUT) {
			LOG_ERROR("DCache clean and invalid time out!\n");
			break;
		}
	} while ((int_status & BIT(0)) == 0);
	retval = target_write_u32(target, REG_DCACHE_INT_CLR, RF_CMD_IRQ_CLR);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_DCACHE_CFG0, ALL_DISABLED);	/* disable all */
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int uwp5661_enter_xip(struct uwp5661_flash_bank *uwp5661_info, struct target *target, uint8_t support_4addr)
{
	int retval = ERROR_OK;
	uint32_t i = 0;
	struct sfc_cmd_des cmd_desc[3];

	create_cmd(&(cmd_desc[0]), CMD_FAST_READ, BYTE_NUM_1, CMD_MODE_WRITE, BIT_MODE_1, SEND_MODE_0);
	create_cmd(&(cmd_desc[1]), 0x0          , (support_4addr == true) ? BYTE_NUM_4 : BYTE_NUM_3,
															CMD_MODE_WRITE, BIT_MODE_1, SEND_MODE_1);
	create_cmd(&(cmd_desc[2]), 0x0          , BYTE_NUM_1, CMD_MODE_HIGHZ, BIT_MODE_1, SEND_MODE_0);

	for (i = 0; i < 3; i++) {
		retval = sfcdrv_set_cmd_data(uwp5661_info, i, &(cmd_desc[i]));
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_write_u32(target, SFC_CMD_BUF0 , uwp5661_info->cmd_info_buf_cache[CMD_BUF_0]);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, SFC_TYPE_BUF0, uwp5661_info->cmd_info_buf_cache[INFO_BUF_0]);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, SFC_TYPE_BUF1, TYPE_BUF_DEFAULT_VALUE);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, SFC_TYPE_BUF2, TYPE_BUF_DEFAULT_VALUE);
	if (retval != ERROR_OK)
		return retval;
	retval = sfcdrv_set_cmd_cfg_reg(uwp5661_info, target, CMD_MODE_READ , BIT_MODE_1, INI_CMD_BUF_7);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_AON_CLK_RF_CGM_MTX_CFG, CGM_MTX_DIV_2);
	if (retval != ERROR_OK)
		return retval;
	/* set CPU clk to xtal MHz */
	retval = target_write_u32(target, REG_AON_CLK_RF_CGM_ARM_CFG, CGM_ARM_XTAL_MHZ);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_AON_CLK_RF_CGM_MTX_CFG, CGM_MTX_DIV_1);
	if (retval != ERROR_OK)
		return retval;

	retval = uwp5661_disable_cache(target);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("Enter XIP");

	return retval;
}

static int uwp5661_exit_xip(struct uwp5661_flash_bank *uwp5661_info, struct target *target)
{
	int retval = ERROR_OK;
	retval = sfcdrv_set_cmd_cfg_reg(uwp5661_info, target, CMD_MODE_WRITE, BIT_MODE_1, INI_CMD_BUF_7);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_AON_CLK_RF_CGM_MTX_CFG, CGM_MTX_DIV_2);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_AON_CLK_RF_CGM_ARM_CFG, CGM_ARM_XTAL_MHZ);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, REG_AON_CLK_RF_CGM_MTX_CFG, CGM_MTX_DIV_1);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("Exit XIP");

	return retval;
}

static int uwp5661_select_xip(struct uwp5661_flash_bank *uwp5661_info, struct target *target,
					uint8_t support_4addr, uint32_t op)
{
	int retval = ERROR_OK;
	retval = target_write_u32(target, SFC_INT_CLR , (1 << SHIFT_INT_CLR));
	if (retval != ERROR_OK)
		return retval;
	if (op == true) {
		retval = uwp5661_enter_xip(uwp5661_info, target, support_4addr);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = uwp5661_exit_xip(uwp5661_info, target);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static enum byte_num uwp5661_flash_addr(uint32_t *addr, uint8_t support_4addr)
{
	uint8_t cmd[4] = {0};
	uint32_t address = *addr;

	cmd[0] = ((address >> 0) & (0xFF));
	cmd[1] = ((address >> 8) & (0xFF));
	cmd[2] = ((address >> 16) & (0xFF));
	cmd[3] = ((address >> 24) & (0xFF));

	if (support_4addr == true) {
		*addr = (cmd[3] << 0) | (cmd[2] << 8) |
				(cmd[1] << 16) | (cmd[0] << 24);
		return BYTE_NUM_4;
	} else {
		*addr = (cmd[2] << 0) | (cmd[1] << 8) | (cmd[0] << 16);
		return BYTE_NUM_3;
	}
}

static int uwp5661_cmd_write(struct uwp5661_flash_bank *uwp5661_info, struct target *target, uint8_t cmd,
			uint32_t *data_out, uint32_t data_len, enum bit_mode bitmode)
{
	int retval = ERROR_OK;
	struct sfc_cmd_des cmd_desc[3];
	enum byte_num byte_num = BYTE_NUM_1;
	uint32_t cmd_idx = 0;

	create_cmd(&(cmd_desc[cmd_idx++]), cmd, BYTE_NUM_1, CMD_MODE_WRITE, bitmode, SEND_MODE_0);

	if (data_len > 8)
		data_len = 8;

	if (data_len > 4) {
		create_cmd(&(cmd_desc[cmd_idx]), data_out[cmd_idx-1], BYTE_NUM_4,
				CMD_MODE_WRITE, bitmode, SEND_MODE_0);
		cmd_idx++;
		data_len = data_len - 4;
	}

	if (data_len > 0) {
		byte_num = BYTE_NUM_1 + (data_len - 1);
		create_cmd(&(cmd_desc[cmd_idx]), data_out[cmd_idx-1], byte_num,
				CMD_MODE_WRITE, bitmode, SEND_MODE_0);
		cmd_idx++;
	}

	retval = uwp5661_read_write(uwp5661_info, target, cmd_desc, cmd_idx, NULL);

	return retval;
}

static int uwp5661_cmd_read(struct uwp5661_flash_bank *uwp5661_info, struct target *target,
			uint8_t cmd, uint32_t *data_in, uint32_t data_len)
{
	int retval = ERROR_OK;
	struct sfc_cmd_des cmd_desc[4];
	enum byte_num byte_num = BYTE_NUM_1;
	uint32_t tmp_buf[2] = {0};
	uint32_t cmd_idx = 0;

	create_cmd(&(cmd_desc[cmd_idx++]), cmd, BYTE_NUM_1, CMD_MODE_WRITE, BIT_MODE_1, SEND_MODE_0);

	if (data_len > 8)
		data_len = 8;

	if (data_len > 4) {
		create_cmd(&(cmd_desc[cmd_idx++]), 0, BYTE_NUM_4, CMD_MODE_READ, BIT_MODE_1, SEND_MODE_0);
		data_len = data_len - 4;
	}

	if (data_len > 0) {
		byte_num = BYTE_NUM_1 + (data_len - 1);
		create_cmd(&(cmd_desc[cmd_idx++]), 0, byte_num  , CMD_MODE_READ, BIT_MODE_1, SEND_MODE_0);
	}

	retval = uwp5661_read_write(uwp5661_info, target, cmd_desc, cmd_idx, tmp_buf);

	if (cmd_idx > 1)
		data_in[0] = (((tmp_buf[0] >> 24) & 0xFF) << 0) |
						(((tmp_buf[0] >> 16) & 0xFF) << 8) |
						(((tmp_buf[0] >> 8) & 0xFF) << 16) |
						(((tmp_buf[0] >> 0) & 0xFF) << 24) ;

	if (cmd_idx > 2)
		data_in[1] = (((tmp_buf[1] >> 24) & 0xFF) << 0) |
						(((tmp_buf[1] >> 16) & 0xFF) << 8) |
						(((tmp_buf[1] >> 8) & 0xFF) << 16) |
						(((tmp_buf[1] >> 0) & 0xFF) << 24) ;

	return retval;
}

static int uwp5661_cmd_poll_bit(struct uwp5661_flash_bank *uwp5661_info, struct target *target, uint32_t timeout,
					uint8_t cmd, uint32_t poll_bit, uint32_t bit_value)
{
	int retval = ERROR_OK;
	uint32_t status = 0;

	do {
		retval = uwp5661_cmd_read(uwp5661_info, target, cmd, &status, 1);
		if (retval != ERROR_OK)
			LOG_ERROR("Flash cmd read failed.");

		status &= 0xFF;
		if (bit_value) {
			if ((status & poll_bit))
				return ERROR_OK;
		} else {
			if ((status & poll_bit) == 0)
				return ERROR_OK;
		}
	} while (timeout--);

	LOG_ERROR("Polling flash status time out!\n");

	return ERROR_FAIL;
}

static int uwp5661_write_enable(struct uwp5661_flash_bank *uwp5661_info, struct target *target)
{
	return uwp5661_cmd_write(uwp5661_info, target, CMD_WRITE_ENABLE, NULL, 0, BIT_MODE_1);
}

static int uwp5661_reset_anyway(struct uwp5661_flash_bank *uwp5661_info, struct target *target)
{
	int retval = ERROR_OK;
	uint32_t i = 0;
	uint32_t dummy_dat = 0;

	retval = uwp5661_cmd_write(uwp5661_info, target, CMD_RSTEN, NULL, 0, BIT_MODE_4);
	if (retval != ERROR_OK)
		return retval;
	retval = uwp5661_cmd_write(uwp5661_info, target, CMD_RST  , NULL, 0, BIT_MODE_4);
	if (retval != ERROR_OK)
		return retval;
	for (i = 0; i < 10; i++) { /* delay */
		retval = target_read_u32(target, SFC_CMD_CFG, &dummy_dat);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = uwp5661_cmd_write(uwp5661_info, target, CMD_RSTEN, NULL, 0, BIT_MODE_1);
	if (retval != ERROR_OK)
		return retval;
	retval = uwp5661_cmd_write(uwp5661_info, target, CMD_RST  , NULL, 0, BIT_MODE_1);
	if (retval != ERROR_OK)
		return retval;
	for (i = 0; i < 10; i++) { /* delay */
		retval = target_read_u32(target, SFC_CMD_CFG, &dummy_dat);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int uwp5661_4addr_enable(struct uwp5661_flash_bank *uwp5661_info, struct target *target)
{
	int retval = ERROR_OK;
	retval = uwp5661_cmd_write(uwp5661_info, target, CMD_ENTER_4ADDR, NULL, 0, BIT_MODE_1);
	if (retval != ERROR_OK)
		return retval;

	return uwp5661_cmd_poll_bit(uwp5661_info, target, SPI_FLASH_ADS_TIMEOUT, CMD_READ_STATUS3, STATUS_ADS, 1);
}

static int uwp5661_4addr_disable(struct uwp5661_flash_bank *uwp5661_info, struct target *target)
{
	int retval = ERROR_OK;
	retval = uwp5661_cmd_write(uwp5661_info, target, CMD_EXIT_4ADDR, NULL, 0, BIT_MODE_1);
	if (retval != ERROR_OK)
		return retval;

	return uwp5661_cmd_poll_bit(uwp5661_info, target, SPI_FLASH_ADS_TIMEOUT, CMD_READ_STATUS3, STATUS_ADS, 0);
}

static int uwp5661_cmd_sector_erase(struct uwp5661_flash_bank *uwp5661_info, struct target *target, uint32_t offset)
{
	int retval = ERROR_OK;
	uint32_t addr = offset * uwp5661_info->flash.sector_size;
	enum byte_num addr_byte_num = uwp5661_flash_addr(&addr, uwp5661_info->flash.support_4addr);

	retval = uwp5661_write_enable(uwp5661_info, target);
	if (retval != ERROR_OK)
		return retval;

	retval = uwp5661_cmd_write(uwp5661_info, target, CMD_SECTOR_ERASE, &addr, addr_byte_num + 1, BIT_MODE_1);
	if (retval != ERROR_OK)
		return retval;

	retval = uwp5661_cmd_poll_bit(uwp5661_info, target, SPI_FLASH_SECTOR_ERASE_TIMEOUT,
			CMD_READ_STATUS1, STATUS_WIP, 0);

	return retval;
}

static int uwp5661_erase(struct flash_bank *bank, int first, int last)
{
	int retval = ERROR_OK;
	struct uwp5661_flash_bank *uwp5661_info = bank->driver_priv;
	struct target *target = bank->target;
	struct uwp_flash *flash = &(uwp5661_info->flash);
	int i = 0;

	memset(uwp5661_info->prev_cmd_info_buf_cache, 0 , sizeof(uwp5661_info->prev_cmd_info_buf_cache));

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted before erasing flash!\n");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = uwp5661_init_sw_csn(target);
	if (retval != ERROR_OK)
		return retval;

	retval = uwp5661_set_sfc_clk(target);
	if (retval != ERROR_OK)
		return retval;

	retval = uwp5661_select_xip(uwp5661_info, target, false, false);
	if (retval != ERROR_OK)
		return retval;

	if (flash->support_4addr == true) {
		retval = uwp5661_4addr_enable(uwp5661_info, target);
		if (retval != ERROR_OK) {
			LOG_ERROR("uwp5661 SPI 4Byte Address mode switching ON failed!\n");
			if (uwp5661_reset_anyway(uwp5661_info, target) != ERROR_OK)
				LOG_ERROR("Flash reset failed.");
			if (uwp5661_select_xip(uwp5661_info, target, false, true) != ERROR_OK)
				LOG_ERROR("Enter xip failed.");
			return retval;
		}
	}

	LOG_INFO("Flash Erase");
	uint32_t cur_ratio = 0;
	uint32_t prev_ratio = 0xFF;
	for (i = first; i <= last; i++) {
		retval = uwp5661_cmd_sector_erase(uwp5661_info, target, i);
		cur_ratio = (i-first+1)*100/(last-first+1);
		if (((cur_ratio/10) != (prev_ratio/10)) && (cur_ratio != 0)) {
			LOG_INFO("\rFlash Erase %3d%%", cur_ratio);
			prev_ratio = cur_ratio;
		}
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	if (flash->support_4addr == true) {
		retval = uwp5661_4addr_disable(uwp5661_info, target);
		if (retval != ERROR_OK) {
			LOG_ERROR("uwp5661 SPI 4Byte Address mode switching OFF failed!\n");
			if (uwp5661_reset_anyway(uwp5661_info, target) != ERROR_OK)
				LOG_ERROR("reset failed.");
		}
	}

	if (uwp5661_select_xip(uwp5661_info, target, false, true) != ERROR_OK)
		LOG_ERROR("select xip failed.");

	return retval;
}

static int uwp5661_write_page(struct uwp5661_flash_bank *uwp5661_info, struct target *target,
			uint32_t data_addr, uint8_t *data_out, uint32_t data_len)
{
	int retval = ERROR_OK;
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t dest_addr = data_addr;
	uint8_t *data = data_out;
	uint32_t data_tmp = 0;
	uint32_t cmd_idx = 0;
	uint32_t piece_cnt = 0;
	enum byte_num byte_num = BYTE_NUM_3;
	struct sfc_cmd_des cmd_desc[CMD_BUF_MAX];

	/* using cs as GPIO bit28 and pull it up, then write cmd and all data in */
	for (i = 0; i < data_len;) {
		cmd_idx = 0;
		piece_cnt = 0;

		if (i == 0) {
			retval = uwp5661_write_enable(uwp5661_info, target);
			if (retval != ERROR_OK)
				return retval;
			retval = uwp5661_force_csn(target, true);
			if (retval != ERROR_OK)
				return retval;

			byte_num = uwp5661_flash_addr(&dest_addr, uwp5661_info->flash.support_4addr);
			/* write cmd and address in */
			create_cmd(&(cmd_desc[cmd_idx++]), CMD_PAGE_PROGRAM, BYTE_NUM_1,
						CMD_MODE_WRITE, BIT_MODE_1, SEND_MODE_0);
			create_cmd(&(cmd_desc[cmd_idx++]), dest_addr       , byte_num  ,
						CMD_MODE_WRITE, BIT_MODE_1, SEND_MODE_0);
		}

		piece_cnt = min((CMD_BUF_MAX - cmd_idx)*4, data_len - i);
		/* write all data in */
		for (j = 0; j < piece_cnt;) {
			if ((piece_cnt - j) >= 4) {
				byte_num = BYTE_NUM_4;
				data_tmp = (data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);

				data = data + 4;
				j = j + 4;
			} else {
				uint32_t tail_bytes = piece_cnt - j;
				byte_num = BYTE_NUM_1 + (tail_bytes - 1);
				switch (tail_bytes) {
					case 1: {
						data_tmp = data[0];
						break;
					}
					case 2: {
						data_tmp = (data[0] << 0) | (data[1] << 8);
						break;
					}
					case 3: {
						data_tmp = (data[0] << 0) | (data[1] << 8) | (data[2] << 16);
						break;
					}
					default:
						break;
				}
				j = piece_cnt;
			}
			create_cmd(&(cmd_desc[cmd_idx++]), data_tmp, byte_num, CMD_MODE_WRITE, BIT_MODE_1, SEND_MODE_0);
		}

		retval = uwp5661_read_write(uwp5661_info, target, cmd_desc, cmd_idx, NULL);
		if (retval != ERROR_OK)
			return retval;

		i = i + piece_cnt;
	}
	retval = uwp5661_force_csn(target, false);
	if (retval != ERROR_OK)
		return retval;
	retval = uwp5661_cmd_poll_bit(uwp5661_info, target, SPI_FLASH_PAGE_PROG_TIMEOUT, CMD_READ_STATUS1, STATUS_WIP, 0);

	return retval;
}

static int uwp5661_write(struct flash_bank *bank, const uint8_t *buffer,
			uint32_t offset, uint32_t count)
{
	int retval = ERROR_OK;
	struct uwp5661_flash_bank *uwp5661_info = bank->driver_priv;
	struct uwp_flash *flash = &(uwp5661_info->flash);
	struct target *target = bank->target;
	uint32_t page_size = flash->page_size;
	uint32_t page_addr = 0;
	uint32_t byte_addr = 0;
	uint32_t chunk_len = 0;
	uint32_t actual    = 0;
	uint32_t data_len  = 0;
	uint32_t space_len = 0;

	memset(uwp5661_info->prev_cmd_info_buf_cache, 0 , sizeof(uwp5661_info->prev_cmd_info_buf_cache));

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted before writing flash!\n");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = uwp5661_init_sw_csn(target);
	if (retval != ERROR_OK)
		return retval;
	retval = uwp5661_set_sfc_clk(target);
	if (retval != ERROR_OK)
		return retval;

	retval = uwp5661_select_xip(uwp5661_info, target, false, false);
	if (retval != ERROR_OK)
		return retval;

	if (flash->support_4addr == true) {
		retval = uwp5661_4addr_enable(uwp5661_info, target);
		if (retval != ERROR_OK) {
			LOG_ERROR("uwp5661 SPI 4Byte Address mode switching ON failed!\n");
			if (uwp5661_reset_anyway(uwp5661_info, target) != ERROR_OK)
				LOG_ERROR("Flash reset failed.");
			if (uwp5661_select_xip(uwp5661_info, target, false, true) != ERROR_OK)
				LOG_ERROR("Enter xip failed.");
			return retval;
		}
	}

	if (offset != 0) {
		page_addr = offset / page_size;
		byte_addr = offset % page_size;
	}

	LOG_INFO("Flash Write");
	uint32_t cur_ratio = 0;
	uint32_t prev_ratio = 0xFF;
	for (actual = 0; actual < count;) {
		data_len = count - actual;
		space_len = page_size - byte_addr;
		chunk_len = min(data_len, space_len);

		retval = uwp5661_write_page(uwp5661_info, target, (page_addr * page_size + byte_addr),
					(uint8_t *)(buffer + actual), chunk_len);

		if (retval != ERROR_OK) {
			LOG_ERROR("Flash write failed\n");
			break;
		}

		page_addr++;
		byte_addr = 0;
		actual += chunk_len;
		cur_ratio = actual*100/count;
		if (((cur_ratio/10) != (prev_ratio/10)) && (cur_ratio != 0)) {
			LOG_INFO("\rFlash Write %3d%%", cur_ratio);
			prev_ratio = cur_ratio;
		}
	}

	if (flash->support_4addr == true) {
		retval = uwp5661_4addr_disable(uwp5661_info, target);
		if (retval != ERROR_OK) {
			LOG_ERROR("uwp5661 SPI 4Byte Address mode switching OFF failed!\n");
			if (uwp5661_reset_anyway(uwp5661_info, target) != ERROR_OK)
				LOG_ERROR("Flash reset failed.");
		}
	}

	if (uwp5661_select_xip(uwp5661_info, target, false, true) != ERROR_OK)
		LOG_ERROR("Enter xip failed.");

	return retval;
}

static int uwp5661_data_read(struct target *target, uint32_t offset,
					uint32_t count, uint8_t *buf)
{
	int retval = ERROR_OK;
	uint32_t i = 0;
	uint32_t addr = offset;
	uint32_t piece_cnt = 0;
	uint8_t tmp_buf[256] = {0};
	uint8_t *data = buf;
	uint32_t cur_ratio = 0;
	uint32_t prev_ratio = 0xFF;

	for (i = 0; i < count;) {
		piece_cnt = min(count - i, 256-(addr%256));

		retval = target_read_memory(target, UWP5661_FLASH_BASE_ADDRESS+(addr&0xFFFFFF00), 4, 64, tmp_buf);

		memcpy(data, tmp_buf+(addr%256), piece_cnt);

		i = i + piece_cnt;
		addr = addr + piece_cnt;
		data = data + piece_cnt;

		cur_ratio = i*100/count;
		if (((cur_ratio/10) != (prev_ratio/10)) && (cur_ratio != 0)) {
			LOG_INFO("\rFlash Read %3d%%", cur_ratio);
			prev_ratio = cur_ratio;
		}
	}

	return retval;
}

static int uwp5661_read(struct flash_bank *bank, uint8_t *buffer,
			uint32_t offset, uint32_t count)
{
	int retval = ERROR_OK;
	struct uwp5661_flash_bank *uwp5661_info = bank->driver_priv;
	struct uwp_flash *flash = &(uwp5661_info->flash);
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted before reading flash!\n");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = uwp5661_init_sw_csn(target);
	if (retval != ERROR_OK)
		return retval;

	retval = uwp5661_set_sfc_clk(target);
	if (retval != ERROR_OK)
		return retval;

	retval = uwp5661_select_xip(uwp5661_info, target, flash->support_4addr, true);
	if (retval != ERROR_OK)
		return retval;

	if (flash->support_4addr == true) {
		retval = uwp5661_4addr_enable(uwp5661_info, target);
		if (retval != ERROR_OK) {
			LOG_ERROR("uwp5661 SPI 4Byte Address mode switching ON failed!\n");
			if (uwp5661_reset_anyway(uwp5661_info, target) != ERROR_OK)
				LOG_ERROR("Flash reset failed.");
			if (uwp5661_select_xip(uwp5661_info, target, false, true) != ERROR_OK)
				LOG_ERROR("Enter xip failed.");
			return retval;
		}
	}

	LOG_INFO("Flash Read");
	retval = uwp5661_data_read(target, offset, count, buffer);
	if (retval != ERROR_OK)
		return retval;

	if (flash->support_4addr == true) {
		retval = uwp5661_4addr_disable(uwp5661_info, target);
		if (retval != ERROR_OK) {
			LOG_ERROR("uwp5661 SPI 4Byte Address mode switching OFF failed!\n");
			if (uwp5661_reset_anyway(uwp5661_info, target) != ERROR_OK)
				LOG_ERROR("Flash reset failed.");
			if (uwp5661_select_xip(uwp5661_info, target, false, true) != ERROR_OK)
				LOG_ERROR("Enter xip failed.");
			return retval;
		}
	}

	retval = uwp5661_select_xip(uwp5661_info, target, false, true);

	return retval;
}

static int uwp5661_probe(struct flash_bank *bank)
{
	int retval = ERROR_OK;
	struct uwp5661_flash_bank *uwp5661_info = bank->driver_priv;
	struct target *target = bank->target;
	struct uwp_flash *flash = &(uwp5661_info->flash);
	struct sfc_cmd_des cmd_desc[2];
	uint32_t read_data;
	const struct flash_device *p = flash_devices;

	bank->base = UWP5661_FLASH_BASE_ADDRESS;

	memset(uwp5661_info->prev_cmd_info_buf_cache, 0 , sizeof(uwp5661_info->prev_cmd_info_buf_cache));
	uwp5661_info->cmd_buf_cache_bitmap = 0xFFF;
	uwp5661_info->sfc_cmd_cfg_cache = 0xFFFFFFFF;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted before probing flash!\n");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = uwp5661_init_sw_csn(target);
	if (retval != ERROR_OK)
		return retval;
	retval = uwp5661_force_csn(target, false);
	if (retval != ERROR_OK)
		return retval;
	retval = uwp5661_set_sfc_clk(target);
	if (retval != ERROR_OK)
		return retval;
	retval = uwp5661_select_xip(uwp5661_info, target, false, false);
	if (retval != ERROR_OK)
		return retval;
	retval = sfcdrv_int_cfg(target, false);
	if (retval != ERROR_OK)
		return retval;
	retval = uwp5661_reset_anyway(uwp5661_info, target);
	if (retval != ERROR_OK)
		return retval;

	/* scan device */
	create_cmd(&(cmd_desc[0]), CMD_READ_ID, BYTE_NUM_1, CMD_MODE_WRITE, BIT_MODE_1, SEND_MODE_0);
	create_cmd(&(cmd_desc[1]),           0, BYTE_NUM_3, CMD_MODE_READ , BIT_MODE_1, SEND_MODE_0);

	retval = uwp5661_read_write(uwp5661_info, target, cmd_desc, 2, &read_data);
	if (retval != ERROR_OK)
		return retval;

	buf_bswap32((uint8_t *)&read_data, (uint8_t *)&read_data, 32);
	for (; p->name ; p++) {
		if (p->device_id == read_data) {
			LOG_INFO("find ID: %x", read_data);
			break;
		}
	}
	if (!p->name) {
		LOG_ERROR("Unsupported ID : 0x%x", read_data);
		return ERROR_FAIL;
	}

	/* config flash and bank*/
	flash->page_size = p->pagesize;
	flash->sector_size = p->sectorsize;
	flash->support_4addr = (p->size_in_bytes > (1<<24)) ? true : false;
	bank->num_sectors = p->size_in_bytes / p->sectorsize;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	for (int i = 0; i < (bank->num_sectors); i++) {
		bank->sectors[i].size = flash->sector_size;
		bank->sectors[i].offset = i * flash->sector_size;
		bank->sectors[i].is_erased = -1;
	}

	retval = uwp5661_select_xip(uwp5661_info, target, false, true);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int uwp5661_auto_probe(struct flash_bank *bank)
{
	return uwp5661_probe(bank);
}

FLASH_BANK_COMMAND_HANDLER(uwp5661_flash_bank_command)
{
	struct uwp5661_flash_bank *uwp5661_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uwp5661_info = malloc(sizeof(struct uwp5661_flash_bank));

	if (!uwp5661_info) {
		LOG_ERROR("No uwp5661_info");
		return ERROR_FAIL;
	}

	bank->driver_priv = uwp5661_info;
	return ERROR_OK;
}

struct flash_driver uwp5661_flash = {
	.name = "uwp5661",
	.flash_bank_command = uwp5661_flash_bank_command,
	.erase = uwp5661_erase,
	.write = uwp5661_write,
	.read = uwp5661_read,
	.probe = uwp5661_probe,
	.auto_probe = uwp5661_auto_probe,
	.free_driver_priv = default_flash_free_driver_priv,
};
