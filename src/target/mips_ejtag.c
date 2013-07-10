/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2009 by David N. Claffey <dnclaffey@gmail.com>          *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mips32.h"
#include "mips64.h"
#include "mips_ejtag.h"

void mips_ejtag_set_instr(struct mips_ejtag *ejtag_info, int new_instr)
{
	struct jtag_tap *tap;

	tap = ejtag_info->tap;
	assert(tap != NULL);
#ifndef BUILD_TARGET64
	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != (uint32_t)new_instr) {
#endif
		struct scan_field field;
		uint8_t t[4];

		field.num_bits = tap->ir_length;
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_instr);
		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, TAP_IDLE);
#ifndef BUILD_TARGET64
	}
#endif
}

int mips_ejtag_get_idcode(struct mips_ejtag *ejtag_info, uint32_t *idcode)
{
	struct scan_field field;
	uint8_t r[4];

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_IDCODE);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = r;

	jtag_add_dr_scan(ejtag_info->tap, 1, &field, TAP_IDLE);

	int retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*idcode = buf_get_u32(field.in_value, 0, 32);

	return ERROR_OK;
}

static int mips_ejtag_get_impcode(struct mips_ejtag *ejtag_info, uint32_t *impcode)
{
	struct scan_field field;
	uint8_t r[4];

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_IMPCODE);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = r;

	jtag_add_dr_scan(ejtag_info->tap, 1, &field, TAP_IDLE);

	int retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*impcode = buf_get_u32(field.in_value, 0, 32);

	return ERROR_OK;
}

void mips_ejtag_add_scan_96(struct mips_ejtag *ejtag_info, uint32_t ctrl, uint32_t data, uint8_t *in_scan_buf)
{
	assert(ejtag_info->tap != NULL);
	struct jtag_tap *tap = ejtag_info->tap;

	struct scan_field field;
	uint8_t out_scan[12];

	/* processor access "all" register 96 bit */
	field.num_bits = 96;

	field.out_value = out_scan;
	buf_set_u32(out_scan, 0, 32, ctrl);
	buf_set_u32(out_scan + 4, 0, 32, data);
	buf_set_u32(out_scan + 8, 0, 32, 0);

	field.in_value = in_scan_buf;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	keep_alive();
}

int mips_ejtag_drscan_64(struct mips_ejtag *ejtag_info, uint64_t *data)
{
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;
	uint8_t t[8], r[8];
	int retval;

	field.num_bits = 64;
	field.out_value = t;
	buf_set_u64(t, 0, field.num_bits, *data);
	field.in_value = r;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u64(field.in_value, 0, 64);

	keep_alive();

	return ERROR_OK;
}

int mips_ejtag_drscan_32(struct mips_ejtag *ejtag_info, uint32_t *data)
{
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;
	uint8_t t[4], r[4];
	int retval;

	field.num_bits = 32;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, *data);
	field.in_value = r;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u32(field.in_value, 0, 32);

	keep_alive();

	return ERROR_OK;
}

void mips_ejtag_drscan_64_out(struct mips_ejtag *ejtag_info, uint64_t data)
{
	uint8_t t[8];
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;

	field.num_bits = 64;
	field.out_value = t;
	buf_set_u64(t, 0, field.num_bits, data);

	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

void mips_ejtag_drscan_32_out(struct mips_ejtag *ejtag_info, uint32_t data)
{
	uint8_t t[4];
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;

	field.num_bits = 32;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, data);

	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

int mips_ejtag_drscan_8(struct mips_ejtag *ejtag_info, uint32_t *data)
{
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;
	uint8_t t[4] = {0, 0, 0, 0}, r[4];
	int retval;

	field.num_bits = 8;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, *data);
	field.in_value = r;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u32(field.in_value, 0, 32);

	return ERROR_OK;
}

void mips_ejtag_drscan_8_out(struct mips_ejtag *ejtag_info, uint8_t data)
{
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;

	field.num_bits = 8;
	field.out_value = &data;
	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

/* Set (to enable) or clear (to disable stepping) the SSt bit (bit 8) in Cp0 Debug reg (reg 23, sel 0) */
int mips64_ejtag_config_step(struct mips_ejtag *ejtag_info, int enable_step)
{
	struct mips64_pracc_queue_info ctx = {.max_code = 7};
	mips64_pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	mips64_pracc_add(&ctx, 0, MIPS64_MFC0(8, 23, 0));			/* move COP0 Debug to $8 */
	mips64_pracc_add(&ctx, 0, MIPS64_ORI(8, 8, 0x0100));			/* set SSt bit in debug reg */
	if (!enable_step)
		mips64_pracc_add(&ctx, 0, MIPS64_XORI(8, 8, 0x0100));		/* clear SSt bit in debug reg */

	mips64_pracc_add(&ctx, 0, MIPS64_MTC0(8, 23, 0));			/* move $8 to COP0 Debug */
	mips64_pracc_add(&ctx, 0, MIPS64_LUI(8, UPPER64_16(ejtag_info->reg8)));		/* restore upper 16 bits  of $8 */
	mips64_pracc_add(&ctx, 0, MIPS64_B(NEG16((ctx.code_count + 1))));			/* jump to start */
	mips64_pracc_add(&ctx, 0, MIPS64_ORI(8, 8, LOWER64_16(ejtag_info->reg8)));	/* restore lower 16 bits of $8 */

	ctx.retval = mips64_pracc_queue_exec(ejtag_info, &ctx, NULL);
exit:
	mips64_pracc_queue_free(&ctx);
	return ctx.retval;
}

/* Set (to enable) or clear (to disable stepping) the SSt bit (bit 8) in Cp0 Debug reg (reg 23, sel 0) */
int mips_ejtag_config_step(struct mips_ejtag *ejtag_info, int enable_step)
{
	struct pracc_queue_info ctx = {.max_code = 7};
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_MFC0(8, 23, 0));			/* move COP0 Debug to $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, 0x0100));			/* set SSt bit in debug reg */
	if (!enable_step)
		pracc_add(&ctx, 0, MIPS32_XORI(8, 8, 0x0100));		/* clear SSt bit in debug reg */

	pracc_add(&ctx, 0, MIPS32_MTC0(8, 23, 0));			/* move $8 to COP0 Debug */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 bits  of $8 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16((ctx.code_count + 1))));			/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 bits of $8 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips_ejtag_enter_debug(struct mips_ejtag *ejtag_info)
{
	uint32_t ejtag_ctrl;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

	/* set debug break bit */
	ejtag_ctrl = ejtag_info->ejtag_ctrl | EJTAG_CTRL_JTAGBRK;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* break bit will be cleared by hardware */
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	LOG_DEBUG("ejtag_ctrl: 0x%8.8" PRIx32 "", ejtag_ctrl);
	if ((ejtag_ctrl & EJTAG_CTRL_BRKST) == 0) {
		LOG_ERROR("Failed to enter Debug Mode!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int mips64_ejtag_exit_debug(struct mips_ejtag *ejtag_info)
{
	uint32_t instr = MIPS64_DRET;
	struct mips64_pracc_queue_info ctx = {.max_code = 1, .pracc_list = &instr, .code_count = 1, .store_count = 0};

	/* execute our dret instruction */
	ctx.retval = mips64_pracc_queue_exec(ejtag_info, &ctx, NULL);

	jtag_add_sleep(1000);
	return ctx.retval;
}

int mips_ejtag_exit_debug(struct mips_ejtag *ejtag_info)
{
	uint32_t instr = MIPS32_DRET;
	struct pracc_queue_info ctx = {.max_code = 1, .pracc_list = &instr, .code_count = 1, .store_count = 0};

	/* execute our dret instruction */
	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);

	/* pic32mx workaround, false pending at low core clock */
	jtag_add_sleep(1000);
	return ctx.retval;
}

int mips_ejtag_init(struct mips_ejtag *ejtag_info)
{
	uint32_t ejtag_version;
	int retval;

	retval = mips_ejtag_get_impcode(ejtag_info, &ejtag_info->impcode);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("impcode: 0x%8.8" PRIx32 "", ejtag_info->impcode);

	/* get ejtag version */
	ejtag_version = ((ejtag_info->impcode >> 29) & 0x07);

	switch (ejtag_version) {
		case 0:
			LOG_DEBUG("EJTAG: Version 1 or 2.0 Detected");
			break;
		case 1:
			LOG_DEBUG("EJTAG: Version 2.5 Detected");
			break;
		case 2:
			LOG_DEBUG("EJTAG: Version 2.6 Detected");
			break;
		case 3:
			LOG_DEBUG("EJTAG: Version 3.1 Detected");
			break;
		case 4:
			LOG_DEBUG("EJTAG: Version 4.1 Detected");
			break;
		case 5:
			LOG_DEBUG("EJTAG: Version 5.1 Detected");
			break;
		default:
			LOG_DEBUG("EJTAG: Unknown Version Detected");
			break;
	}
	LOG_DEBUG("EJTAG: features:%s%s%s%s%s%s%s",
		ejtag_info->impcode & EJTAG_IMP_R3K ? " R3k" : " R4k",
		ejtag_info->impcode & EJTAG_IMP_DINT ? " DINT" : "",
		ejtag_info->impcode & (1 << 22) ? " ASID_8" : "",
		ejtag_info->impcode & (1 << 21) ? " ASID_6" : "",
		ejtag_info->impcode & EJTAG_IMP_MIPS16 ? " MIPS16" : "",
		ejtag_info->impcode & EJTAG_IMP_NODMA ? " noDMA" : " DMA",
		ejtag_info->impcode & EJTAG_DCR_MIPS64  ? " MIPS64" : " MIPS32");

	if ((ejtag_info->impcode & EJTAG_IMP_NODMA) == 0)
		LOG_DEBUG("EJTAG: DMA Access Mode Support Enabled");

	/* set initial state for ejtag control reg */
	ejtag_info->ejtag_ctrl = EJTAG_CTRL_ROCC | EJTAG_CTRL_PRACC | EJTAG_CTRL_PROBEN | EJTAG_CTRL_SETDEV;
	ejtag_info->fast_access_save = -1;

	return ERROR_OK;
}

int mips64_ejtag_fastdata_scan(struct mips_ejtag *ejtag_info, int write_t, uint64_t *data)
{
	struct jtag_tap *tap;

	tap = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field fields[2];
	uint8_t spracc = 0;
	uint8_t t[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	/* fastdata 1-bit register */
	fields[0].num_bits = 1;
	fields[0].out_value = &spracc;
	fields[0].in_value = NULL;

	/* processor access data register 32 bit */
	fields[1].num_bits = 64;
	fields[1].out_value = t;

	if (write_t) {
		fields[1].in_value = NULL;
		buf_set_u64(t, 0, 64, *data);
	} else
		fields[1].in_value = (void *) data;

	jtag_add_dr_scan(tap, 2, fields, TAP_IDLE);

	if (!write_t && data)
		jtag_add_callback(mips_le_to_h_u64,
				  (jtag_callback_data_t) data);

	keep_alive();

	return ERROR_OK;
}

int mips_ejtag_fastdata_scan(struct mips_ejtag *ejtag_info, int write_t, uint32_t *data)
{
	struct jtag_tap *tap;

	tap = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field fields[2];
	uint8_t spracc = 0;
	uint8_t t[4] = {0, 0, 0, 0};

	/* fastdata 1-bit register */
	fields[0].num_bits = 1;
	fields[0].out_value = &spracc;
	fields[0].in_value = NULL;

	/* processor access data register 32 bit */
	fields[1].num_bits = 32;
	fields[1].out_value = t;

	if (write_t) {
		fields[1].in_value = NULL;
		buf_set_u32(t, 0, 32, *data);
	} else
		fields[1].in_value = (void *) data;

	jtag_add_dr_scan(tap, 2, fields, TAP_IDLE);

	if (!write_t && data)
		jtag_add_callback(mips_le_to_h_u32,
			(jtag_callback_data_t) data);

	keep_alive();

	return ERROR_OK;
}
