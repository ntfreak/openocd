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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mips32.h"
#include "mips_ejtag.h"
#include "mips32_dmaacc.h"
#include "mips64.h"
#include "mips64_pracc.h"

void mips_ejtag_set_instr(struct mips_ejtag *ejtag_info, uint32_t new_instr)
{
	assert(ejtag_info->tap != NULL);
	struct jtag_tap *tap = ejtag_info->tap;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr) {

		struct scan_field field;
		field.num_bits = tap->ir_length;

		uint8_t t[4] = { 0 };
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_instr);

		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, TAP_IDLE);
	}
}

int mips_ejtag_get_idcode(struct mips_ejtag *ejtag_info)
{
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_IDCODE);

	ejtag_info->idcode = 0;
	return mips_ejtag_drscan_32(ejtag_info, &ejtag_info->idcode);
}

static int mips_ejtag_get_impcode(struct mips_ejtag *ejtag_info)
{
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_IMPCODE);

	ejtag_info->impcode = 0;
	return mips_ejtag_drscan_32(ejtag_info, &ejtag_info->impcode);
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

	if (tap == NULL)
		return ERROR_FAIL;
	struct scan_field field;
	uint8_t t[8] = { 0 }, r[8];
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

static void mips_ejtag_drscan_32_queued(struct mips_ejtag *ejtag_info,
		uint32_t data_out, uint8_t *data_in)
{
	assert(ejtag_info->tap != NULL);
	struct jtag_tap *tap = ejtag_info->tap;

	struct scan_field field;
	field.num_bits = 32;

	uint8_t scan_out[4] = { 0 };
	field.out_value = scan_out;
	buf_set_u32(scan_out, 0, field.num_bits, data_out);

	field.in_value = data_in;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	keep_alive();
}

int mips_ejtag_drscan_32(struct mips_ejtag *ejtag_info, uint32_t *data)
{
	uint8_t scan_in[4];
	mips_ejtag_drscan_32_queued(ejtag_info, *data, scan_in);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u32(scan_in, 0, 32);
	return ERROR_OK;
}

void mips_ejtag_drscan_32_out(struct mips_ejtag *ejtag_info, uint32_t data)
{
	mips_ejtag_drscan_32_queued(ejtag_info, data, NULL);
}

int mips_ejtag_drscan_8(struct mips_ejtag *ejtag_info, uint8_t *data)
{
	assert(ejtag_info->tap != NULL);
	struct jtag_tap *tap = ejtag_info->tap;

	struct scan_field field;
	field.num_bits = 8;

	field.out_value = data;
	field.in_value = data;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}
	return ERROR_OK;
}

void mips_ejtag_drscan_8_out(struct mips_ejtag *ejtag_info, uint8_t data)
{
	assert(ejtag_info->tap != NULL);
	struct jtag_tap *tap = ejtag_info->tap;

	struct scan_field field;
	field.num_bits = 8;

	field.out_value = &data;
	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

/* Set (to enable) or clear (to disable stepping) the SSt bit (bit 8) in Cp0 Debug reg (reg 23, sel 0) */
int mips_ejtag_config_step(struct mips_ejtag *ejtag_info, int enable_step)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 8, 23, 0));			/* move COP0 Debug to $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 8, 0x0100));			/* set SSt bit in debug reg */
	if (!enable_step)
		pracc_add(&ctx, 0, MIPS32_XORI(ctx.isa, 8, 8, 0x0100));		/* clear SSt bit in debug reg */

	pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 8, 23, 0));			/* move $8 to COP0 Debug */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 8, UPPER16(ejtag_info->reg8)));	/* restore upper 16 bits  of $8 */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 8, LOWER16(ejtag_info->reg8))); /* restore lower 16 bits of $8 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

/*
 * Disable memory protection for 0xFF20.0000–0xFF3F.FFFF
 * It is needed by EJTAG 1.5-2.0, especially for BMIPS CPUs
 * For example bcm7401 and others. At leas on some
 * CPUs, DebugMode wont start if this bit is not removed.
 */
static int disable_dcr_mp(struct mips_ejtag *ejtag_info)
{
	uint32_t dcr;
	int retval;

	retval = mips32_dmaacc_read_mem(ejtag_info, EJTAG_DCR, 4, 1, &dcr);
	if (retval != ERROR_OK)
		goto error;

	dcr &= ~EJTAG_DCR_MP;
	retval = mips32_dmaacc_write_mem(ejtag_info, EJTAG_DCR, 4, 1, &dcr);
	if (retval != ERROR_OK)
		goto error;
	return ERROR_OK;
error:
	LOG_ERROR("Failed to remove DCR MPbit!");
	return retval;
}

int mips_ejtag_enter_debug(struct mips_ejtag *ejtag_info)
{
	uint32_t ejtag_ctrl;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

	if (ejtag_info->ejtag_version == EJTAG_VERSION_20) {
		if (disable_dcr_mp(ejtag_info) != ERROR_OK)
			goto error;
	}

	/* set debug break bit */
	ejtag_ctrl = ejtag_info->ejtag_ctrl | EJTAG_CTRL_JTAGBRK;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* break bit will be cleared by hardware */
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	LOG_DEBUG("ejtag_ctrl: 0x%8.8" PRIx32 "", ejtag_ctrl);
	if ((ejtag_ctrl & EJTAG_CTRL_BRKST) == 0)
		goto error;

	return ERROR_OK;
error:
	LOG_ERROR("Failed to enter Debug Mode!");
	return ERROR_FAIL;
}

int mips_ejtag_exit_debug(struct mips_ejtag *ejtag_info)
{
	pa_list pracc_list = {.instr = MIPS32_DRET(ejtag_info->isa), .addr = 0};
	struct pracc_queue_info ctx = {.max_code = 1, .pracc_list = &pracc_list, .code_count = 1, .store_count = 0};

	/* execute our dret instruction */
	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 0); /* shift out instr, omit last check */

	/* pic32mx workaround, false pending at low core clock */
	jtag_add_sleep(1000);
	return ctx.retval;
}

/* mips_ejtag_init_mmr - assign Memory-Mapped Registers depending
 *			on EJTAG version.
 */
static void mips_ejtag_init_mmr(struct mips_ejtag *ejtag_info)
{
	if (ejtag_info->ejtag_version == EJTAG_VERSION_20) {
		ejtag_info->ejtag_ibs_addr	= EJTAG_V20_IBS;
		ejtag_info->ejtag_iba0_addr	= EJTAG_V20_IBA0;
		ejtag_info->ejtag_ibc_offs	= EJTAG_V20_IBC_OFFS;
		ejtag_info->ejtag_ibm_offs	= EJTAG_V20_IBM_OFFS;

		ejtag_info->ejtag_dbs_addr	= EJTAG_V20_DBS;
		ejtag_info->ejtag_dba0_addr	= EJTAG_V20_DBA0;
		ejtag_info->ejtag_dbc_offs	= EJTAG_V20_DBC_OFFS;
		ejtag_info->ejtag_dbm_offs	= EJTAG_V20_DBM_OFFS;
		ejtag_info->ejtag_dbv_offs	= EJTAG_V20_DBV_OFFS;

		ejtag_info->ejtag_iba_step_size	= EJTAG_V20_IBAn_STEP;
		ejtag_info->ejtag_dba_step_size	= EJTAG_V20_DBAn_STEP;
	} else {
		ejtag_info->ejtag_ibs_addr	= EJTAG_V25_IBS;
		ejtag_info->ejtag_iba0_addr	= EJTAG_V25_IBA0;
		ejtag_info->ejtag_ibm_offs	= EJTAG_V25_IBM_OFFS;
		ejtag_info->ejtag_ibasid_offs	= EJTAG_V25_IBASID_OFFS;
		ejtag_info->ejtag_ibc_offs	= EJTAG_V25_IBC_OFFS;

		ejtag_info->ejtag_dbs_addr	= EJTAG_V25_DBS;
		ejtag_info->ejtag_dba0_addr	= EJTAG_V25_DBA0;
		ejtag_info->ejtag_dbm_offs	= EJTAG_V25_DBM_OFFS;
		ejtag_info->ejtag_dbasid_offs	= EJTAG_V25_DBASID_OFFS;
		ejtag_info->ejtag_dbc_offs	= EJTAG_V25_DBC_OFFS;
		ejtag_info->ejtag_dbv_offs	= EJTAG_V25_DBV_OFFS;

		ejtag_info->ejtag_iba_step_size	= EJTAG_V25_IBAn_STEP;
		ejtag_info->ejtag_dba_step_size	= EJTAG_V25_DBAn_STEP;
	}
}

static void ejtag_v20_print_imp(struct mips_ejtag *ejtag_info)
{
	LOG_DEBUG("EJTAG v2.0: features:%s%s%s%s%s%s%s%s",
		EJTAG_IMP_HAS(EJTAG_V20_IMP_SDBBP) ? " SDBBP_SPECIAL2" : " SDBBP",
		EJTAG_IMP_HAS(EJTAG_V20_IMP_EADDR_NO32BIT) ? " EADDR>32bit" : " EADDR=32bit",
		EJTAG_IMP_HAS(EJTAG_V20_IMP_COMPLEX_BREAK) ? " COMPLEX_BREAK" : "",
		EJTAG_IMP_HAS(EJTAG_V20_IMP_DCACHE_COH) ? " DCACHE_COH" : " DCACHE_NOT_COH",
		EJTAG_IMP_HAS(EJTAG_V20_IMP_ICACHE_COH) ? " ICACHE_COH" : " ICACHE_NOT_COH",
		EJTAG_IMP_HAS(EJTAG_V20_IMP_NOPB) ? " noPB" : " PB",
		EJTAG_IMP_HAS(EJTAG_V20_IMP_NODB) ? " noDB" : " DB",
		EJTAG_IMP_HAS(EJTAG_V20_IMP_NOIB) ? " noIB" : " IB");
	LOG_DEBUG("EJTAG v2.0: Break Channels: %" PRIu8,
		(uint8_t)((ejtag_info->impcode >> EJTAG_V20_IMP_BCHANNELS_SHIFT) &
		EJTAG_V20_IMP_BCHANNELS_MASK));
}

static void ejtag_v26_print_imp(struct mips_ejtag *ejtag_info)
{
	LOG_DEBUG("EJTAG v2.6: features:%s%s",
		EJTAG_IMP_HAS(EJTAG_V26_IMP_R3K) ? " R3k" : " R4k",
		EJTAG_IMP_HAS(EJTAG_V26_IMP_DINT) ? " DINT" : "");
}

static void ejtag_main_print_imp(struct mips_ejtag *ejtag_info)
{
	LOG_DEBUG("EJTAG main: features:%s%s%s%s%s",
		EJTAG_IMP_HAS(EJTAG_IMP_ASID8) ? " ASID_8" : "",
		EJTAG_IMP_HAS(EJTAG_IMP_ASID6) ? " ASID_6" : "",
		EJTAG_IMP_HAS(EJTAG_IMP_MIPS16) ? " MIPS16" : "",
		EJTAG_IMP_HAS(EJTAG_IMP_NODMA) ? " noDMA" : " DMA",
		EJTAG_IMP_HAS(EJTAG_IMP_MIPS64) ? " MIPS64" : " MIPS32");

	switch (ejtag_info->ejtag_version) {
		case EJTAG_VERSION_20:
			ejtag_v20_print_imp(ejtag_info);
			break;
		case EJTAG_VERSION_25:
		case EJTAG_VERSION_26:
		case EJTAG_VERSION_31:
		case EJTAG_VERSION_41:
		case EJTAG_VERSION_51:
			ejtag_v26_print_imp(ejtag_info);
			break;
		default:
			break;
	}
}

int mips_ejtag_init(struct mips_ejtag *ejtag_info)
{
	int retval = mips_ejtag_get_impcode(ejtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("impcode read failed");
		return retval;
	}

	/* get ejtag version */
	ejtag_info->ejtag_version = ((ejtag_info->impcode >> 29) & 0x07);

	switch (ejtag_info->ejtag_version) {
		case EJTAG_VERSION_20:
			LOG_DEBUG("EJTAG: Version 1 or 2.0 Detected");
			break;
		case EJTAG_VERSION_25:
			LOG_DEBUG("EJTAG: Version 2.5 Detected");
			break;
		case EJTAG_VERSION_26:
			LOG_DEBUG("EJTAG: Version 2.6 Detected");
			break;
		case EJTAG_VERSION_31:
			LOG_DEBUG("EJTAG: Version 3.1 Detected");
			break;
		case EJTAG_VERSION_41:
			LOG_DEBUG("EJTAG: Version 4.1 Detected");
			break;
		case EJTAG_VERSION_51:
			LOG_DEBUG("EJTAG: Version 5.1 Detected");
			break;
		default:
			LOG_DEBUG("EJTAG: Unknown Version Detected");
			break;
	}
	ejtag_main_print_imp(ejtag_info);

	if ((ejtag_info->impcode & EJTAG_IMP_NODMA) == 0) {
		LOG_DEBUG("EJTAG: DMA Access Mode detected. Disabling to "
			  "workaround current broken code.");
		ejtag_info->impcode |= EJTAG_IMP_NODMA;
	}

	ejtag_info->ejtag_ctrl = EJTAG_CTRL_PRACC | EJTAG_CTRL_PROBEN;

	if (ejtag_info->ejtag_version != EJTAG_VERSION_20)
		ejtag_info->ejtag_ctrl |= EJTAG_CTRL_ROCC | EJTAG_CTRL_SETDEV;

	ejtag_info->fast_access_save = -1;

	mips_ejtag_init_mmr(ejtag_info);

	return ERROR_OK;
}

int mips_ejtag_fastdata_scan(struct mips_ejtag *ejtag_info, int write_t, uint32_t *data)
{
	assert(ejtag_info->tap != NULL);
	struct jtag_tap *tap = ejtag_info->tap;

	struct scan_field fields[2];

	/* fastdata 1-bit register */
	fields[0].num_bits = 1;

	uint8_t spracc = 0;
	fields[0].out_value = &spracc;
	fields[0].in_value = NULL;

	/* processor access data register 32 bit */
	fields[1].num_bits = 32;

	uint8_t t[4] = {0, 0, 0, 0};
	fields[1].out_value = t;

	if (write_t) {
		fields[1].in_value = NULL;
		buf_set_u32(t, 0, 32, *data);
	} else
		fields[1].in_value = (uint8_t *) data;

	jtag_add_dr_scan(tap, 2, fields, TAP_IDLE);

	if (!write_t && data)
		jtag_add_callback(mips_le_to_h_u32,
			(jtag_callback_data_t) data);

	keep_alive();

	return ERROR_OK;
}

int mips64_ejtag_config_step(struct mips_ejtag *ejtag_info, bool enable_step)
{
	const uint32_t code_enable[] = {
		MIPS64_MTC0(1, 31, 0),		    /* move $1 to COP0 DeSave */
		MIPS64_MFC0(1, 23, 0),		    /* move COP0 Debug to $1 */
		MIPS64_ORI(1, 1, 0x0100),		 /* set SSt bit in debug reg */
		MIPS64_MTC0(1, 23, 0),		    /* move $1 to COP0 Debug */
		MIPS64_B(NEG16(5)),
		MIPS64_MFC0(1, 31, 0),		    /* move COP0 DeSave to $1 */
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};

	const uint32_t code_disable[] = {
		MIPS64_MTC0(15, 31, 0),                           /* move $15 to COP0 DeSave */
		MIPS64_LUI(15, UPPER16(MIPS64_PRACC_STACK)),     /* $15 = MIPS64_PRACC_STACK */
		MIPS64_ORI(15, 15, LOWER16(MIPS64_PRACC_STACK)),
		MIPS64_SD(1, 0, 15),                              /* sw $1,($15) */
		MIPS64_SD(2, 0, 15),                              /* sw $2,($15) */
		MIPS64_MFC0(1, 23, 0),                            /* move COP0 Debug to $1 */
		MIPS64_LUI(2, 0xFFFF),                           /* $2 = 0xfffffeff */
		MIPS64_ORI(2, 2, 0xFEFF),
		MIPS64_AND(1, 1, 2),
		MIPS64_MTC0(1, 23, 0),                            /* move $1 to COP0 Debug */
		MIPS64_LD(2, 0, 15),
		MIPS64_LD(1, 0, 15),
		MIPS64_SYNC,
		MIPS64_B(NEG16(14)),
		MIPS64_MFC0(15, 31, 0),                           /* move COP0 DeSave to $15 */
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};
	const uint32_t *code = enable_step ? code_enable : code_disable;
	unsigned code_len = enable_step ? ARRAY_SIZE(code_enable) :
					  ARRAY_SIZE(code_disable);

	return mips64_pracc_exec(ejtag_info,
				 code_len, code, 0, NULL, 0, NULL);
}

int mips64_ejtag_exit_debug(struct mips_ejtag *ejtag_info)
{
	const uint32_t code[] = {
		MIPS64_DRET,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
		MIPS64_NOP,
	};
	LOG_DEBUG("enter mips64_pracc_exec");
	return mips64_pracc_exec(ejtag_info,
				 ARRAY_SIZE(code), code, 0, NULL, 0, NULL);
}

int mips64_ejtag_fastdata_scan(struct mips_ejtag *ejtag_info, bool write_t, uint64_t *data)
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

	/* processor access data register 64 bit */
	fields[1].num_bits = 64;
	fields[1].out_value = t;

	if (write_t) {
		fields[1].in_value = NULL;
		buf_set_u64(t, 0, 64, *data);
	} else
		fields[1].in_value = (uint8_t *) data;

	jtag_add_dr_scan(tap, 2, fields, TAP_IDLE);

	if (!write_t && data)
		jtag_add_callback(mips_le_to_h_u64,
			(jtag_callback_data_t) data);
	keep_alive();

	return ERROR_OK;
}
