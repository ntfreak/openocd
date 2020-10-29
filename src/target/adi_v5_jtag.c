/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin
 *   lundin@mlu.mine.nu
 *
 *   Copyright (C) 2008 by Spencer Oliver
 *   spen@spen-soft.co.uk
 *
 *   Copyright (C) 2009 by Oyvind Harboe
 *   oyvind.harboe@zylin.com
 *
 *   Copyright (C) 2009-2010 by David Brownell
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
 * @file
 * This file implements JTAG transport support for cores implementing
 the ARM Debug Interface version 5 (ADIv5).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>
#include <helper/list.h>
#include <jtag/swd.h>

/*#define DEBUG_WAIT*/

/* JTAG instructions/registers for JTAG-DP and SWJ-DP */
#define JTAG_DP_ABORT		0x8
#define JTAG_DP_DPACC		0xA
#define JTAG_DP_APACC		0xB
#define JTAG_DP_IDCODE		0xE

/* three-bit ACK values for DPACC and APACC reads */
#define JTAG_ACK_OK_FAULT	0x2
#define JTAG_ACK_WAIT		0x1

static int jtag_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack);

#ifdef DEBUG_WAIT
static const char *dap_reg_name(int instr, int reg_addr)
{
	char *reg_name = "UNK";

	if (instr == JTAG_DP_DPACC) {
		switch (reg_addr) {
		case DP_ABORT:
			reg_name =  "ABORT";
			break;
		case DP_CTRL_STAT:
			reg_name =  "CTRL/STAT";
			break;
		case DP_SELECT:
			reg_name = "SELECT";
			break;
		case DP_RDBUFF:
			reg_name =  "RDBUFF";
			break;
		case DP_DLCR:
			reg_name =  "DLCR";
			break;
		default:
			reg_name = "UNK";
			break;
		}
	}

	if (instr == JTAG_DP_APACC) {
		switch (reg_addr) {
		case MEM_AP_REG_CSW:
			reg_name = "CSW";
			break;
		case MEM_AP_REG_TAR:
			reg_name = "TAR";
			break;
		case MEM_AP_REG_DRW:
			reg_name = "DRW";
			break;
		case MEM_AP_REG_BD0:
			reg_name = "BD0";
			break;
		case MEM_AP_REG_BD1:
			reg_name = "BD1";
			break;
		case MEM_AP_REG_BD2:
			reg_name = "BD2";
			break;
		case MEM_AP_REG_BD3:
			reg_name = "BD3";
			break;
		case MEM_AP_REG_CFG:
			reg_name = "CFG";
			break;
		case MEM_AP_REG_BASE:
			reg_name = "BASE";
			break;
		case AP_REG_IDR:
			reg_name = "IDR";
			break;
		default:
			reg_name = "UNK";
			break;
		}
	}

	return reg_name;
}
#endif

struct dap_cmd {
	struct list_head lh;
	uint8_t instr;
	uint8_t reg_addr;
	uint8_t RnW;
	uint8_t *invalue;
	uint8_t ack;
	uint32_t memaccess_tck;
	uint32_t dp_select;

	struct scan_field fields[2];
	uint8_t out_addr_buf;
	uint8_t invalue_buf[4];
	uint8_t outvalue_buf[4];
};

#define MAX_DAP_COMMAND_NUM 65536

struct dap_cmd_pool {
	struct list_head lh;
	struct dap_cmd cmd;
};

static void log_dap_cmd(const char *header, struct dap_cmd *el)
{
#ifdef DEBUG_WAIT
	LOG_DEBUG("%s: %2s %6s %5s 0x%08x 0x%08x %2s", header,
		el->instr == JTAG_DP_APACC ? "AP" : "DP",
		dap_reg_name(el->instr, el->reg_addr),
		el->RnW == DPAP_READ ? "READ" : "WRITE",
		buf_get_u32(el->outvalue_buf, 0, 32),
		buf_get_u32(el->invalue, 0, 32),
		el->ack == JTAG_ACK_OK_FAULT ? "OK" :
			(el->ack == JTAG_ACK_WAIT ? "WAIT" : "INVAL"));
#endif
}

static int jtag_limit_queue_size(struct adiv5_dap *dap)
{
	if (dap->cmd_pool_size < MAX_DAP_COMMAND_NUM)
		return ERROR_OK;

	return dap_run(dap);
}

static struct dap_cmd *dap_cmd_new(struct adiv5_dap *dap, uint8_t instr,
		uint8_t reg_addr, uint8_t RnW,
		uint8_t *outvalue, uint8_t *invalue,
		uint32_t memaccess_tck)
{

	struct dap_cmd_pool *pool = NULL;

	if (list_empty(&dap->cmd_pool)) {
		pool = calloc(1, sizeof(struct dap_cmd_pool));
		if (pool == NULL)
			return NULL;
	} else {
		pool = list_first_entry(&dap->cmd_pool, struct dap_cmd_pool, lh);
		list_del(&pool->lh);
	}

	INIT_LIST_HEAD(&pool->lh);
	dap->cmd_pool_size++;

	struct dap_cmd *cmd = &pool->cmd;
	INIT_LIST_HEAD(&cmd->lh);
	cmd->instr = instr;
	cmd->reg_addr = reg_addr;
	cmd->RnW = RnW;
	if (outvalue != NULL)
		memcpy(cmd->outvalue_buf, outvalue, 4);
	cmd->invalue = (invalue != NULL) ? invalue : cmd->invalue_buf;
	cmd->memaccess_tck = memaccess_tck;

	return cmd;
}

static void dap_cmd_release(struct adiv5_dap *dap, struct dap_cmd *cmd)
{
	struct dap_cmd_pool *pool = container_of(cmd, struct dap_cmd_pool, cmd);
	if (dap->cmd_pool_size > MAX_DAP_COMMAND_NUM)
		free(pool);
	else
		list_add(&pool->lh, &dap->cmd_pool);

	dap->cmd_pool_size--;
}

static void flush_journal(struct adiv5_dap *dap, struct list_head *lh)
{
	struct dap_cmd *el, *tmp;

	list_for_each_entry_safe(el, tmp, lh, lh) {
		list_del(&el->lh);
		dap_cmd_release(dap, el);
	}
}

static void jtag_quit(struct adiv5_dap *dap)
{
	struct dap_cmd_pool *el, *tmp;
	struct list_head *lh = &dap->cmd_pool;

	list_for_each_entry_safe(el, tmp, lh, lh) {
		list_del(&el->lh);
		free(el);
	}
}

/***************************************************************************
 *
 * DPACC and APACC scanchain access through JTAG-DP (or SWJ-DP)
 *
***************************************************************************/

static int adi_jtag_dp_scan_cmd(struct adiv5_dap *dap, struct dap_cmd *cmd, uint8_t *ack)
{
	struct jtag_tap *tap = dap->tap;
	int retval;

	retval = arm_jtag_set_instr(tap, cmd->instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	/* Scan out a read or write operation using some DP or AP register.
	 * For APACC access with any sticky error flag set, this is discarded.
	 */
	cmd->fields[0].num_bits = 3;
	buf_set_u32(&cmd->out_addr_buf, 0, 3, ((cmd->reg_addr >> 1) & 0x6) | (cmd->RnW & 0x1));
	cmd->fields[0].out_value = &cmd->out_addr_buf;
	cmd->fields[0].in_value = (ack != NULL) ? ack : &cmd->ack;

	/* NOTE: if we receive JTAG_ACK_WAIT, the previous operation did not
	 * complete; data we write is discarded, data we read is unpredictable.
	 * When overrun detect is active, STICKYORUN is set.
	 */

	cmd->fields[1].num_bits = 32;
	cmd->fields[1].out_value = cmd->outvalue_buf;
	cmd->fields[1].in_value = cmd->invalue;

	jtag_add_dr_scan(tap, 2, cmd->fields, TAP_IDLE);

	/* Add specified number of tck clocks after starting memory bus
	 * access, giving the hardware time to complete the access.
	 * They provide more time for the (MEM) AP to complete the read ...
	 * See "Minimum Response Time" for JTAG-DP, in the ADIv5 spec.
	 */
	if (cmd->instr == JTAG_DP_APACC) {
		if (((cmd->reg_addr == MEM_AP_REG_DRW)
			|| ((cmd->reg_addr & 0xF0) == MEM_AP_REG_BD0))
			&& (cmd->memaccess_tck != 0))
			jtag_add_runtest(cmd->memaccess_tck, TAP_IDLE);
	}

	return ERROR_OK;
}

static int adi_jtag_dp_scan_cmd_sync(struct adiv5_dap *dap, struct dap_cmd *cmd, uint8_t *ack)
{
	int retval;

	retval = adi_jtag_dp_scan_cmd(dap, cmd, ack);
	if (retval != ERROR_OK)
		return retval;

	return jtag_execute_queue();
}

/**
 * Scan DPACC or APACC using target ordered uint8_t buffers.  No endianness
 * conversions are performed.  See section 4.4.3 of the ADIv5 spec, which
 * discusses operations which access these registers.
 *
 * Note that only one scan is performed.  If RnW is set, a separate scan
 * will be needed to collect the data which was read; the "invalue" collects
 * the posted result of a preceding operation, not the current one.
 *
 * @param dap the DAP
 * @param instr JTAG_DP_APACC (AP access) or JTAG_DP_DPACC (DP access)
 * @param reg_addr two significant bits; A[3:2]; for APACC access, the
 *	SELECT register has more addressing bits.
 * @param RnW false iff outvalue will be written to the DP or AP
 * @param outvalue points to a 32-bit (little-endian) integer
 * @param invalue NULL, or points to a 32-bit (little-endian) integer
 * @param ack points to where the three bit JTAG_ACK_* code will be stored
 * @param memaccess_tck number of idle cycles to add after AP access
 */

static int adi_jtag_dp_scan(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint8_t *outvalue, uint8_t *invalue,
		uint32_t memaccess_tck, uint8_t *ack)
{
	struct dap_cmd *cmd;
	int retval;

	cmd = dap_cmd_new(dap, instr, reg_addr, RnW, outvalue, invalue, memaccess_tck);
	if (cmd != NULL)
		cmd->dp_select = dap->select;
	else
		return ERROR_JTAG_DEVICE_ERROR;

	retval = adi_jtag_dp_scan_cmd(dap, cmd, ack);
	if (retval == ERROR_OK)
		list_add_tail(&cmd->lh,	&dap->cmd_journal);

	return retval;
}

/**
 * Scan DPACC or APACC out and in from host ordered uint32_t buffers.
 * This is exactly like adi_jtag_dp_scan(), except that endianness
 * conversions are performed (so the types of invalue and outvalue
 * must be different).
 */
static int adi_jtag_dp_scan_u32(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint32_t outvalue, uint32_t *invalue,
		uint32_t memaccess_tck, uint8_t *ack)
{
	uint8_t out_value_buf[4];
	int retval;

	buf_set_u32(out_value_buf, 0, 32, outvalue);

	retval = adi_jtag_dp_scan(dap, instr, reg_addr, RnW,
			out_value_buf, (uint8_t *)invalue, memaccess_tck, ack);
	if (retval != ERROR_OK)
		return retval;

	if (invalue)
		jtag_add_callback(arm_le_to_h_u32,
				(jtag_callback_data_t) invalue);

	return retval;
}

static int adi_jtag_finish_read(struct adiv5_dap *dap)
{
	int retval = ERROR_OK;

	if (dap->last_read != NULL) {
		retval = adi_jtag_dp_scan_u32(dap, JTAG_DP_DPACC,
				DP_RDBUFF, DPAP_READ, 0, dap->last_read, 0, NULL);
		dap->last_read = NULL;
	}

	return retval;
}

static int adi_jtag_scan_inout_check_u32(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint32_t outvalue, uint32_t *invalue, uint32_t memaccess_tck)
{
	int retval;

	/* Issue the read or write */
	retval = adi_jtag_dp_scan_u32(dap, instr, reg_addr,
			RnW, outvalue, NULL, memaccess_tck, NULL);
	if (retval != ERROR_OK)
		return retval;

	/* For reads,  collect posted value; RDBUFF has no other effect.
	 * Assumes read gets acked with OK/FAULT, and CTRL_STAT says "OK".
	 */
	if ((RnW == DPAP_READ) && (invalue != NULL)) {
		retval = adi_jtag_dp_scan_u32(dap, JTAG_DP_DPACC,
				DP_RDBUFF, DPAP_READ, 0, invalue, 0, NULL);
		if (retval != ERROR_OK)
			return retval;
	}

	return jtag_execute_queue();
}

static int jtagdp_overrun_check(struct adiv5_dap *dap)
{
	int retval;
	struct dap_cmd *el, *tmp, *prev = NULL;
	int found_wait = 0;
	int64_t time_now;
	LIST_HEAD(replay_list);

	/* make sure all queued transactions are complete */
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		goto done;

	/* skip all completed transactions up to the first WAIT */
	list_for_each_entry(el, &dap->cmd_journal, lh) {
		if (el->ack == JTAG_ACK_OK_FAULT) {
			log_dap_cmd("LOG", el);
		} else if (el->ack == JTAG_ACK_WAIT) {
			found_wait = 1;
			break;
		} else {
			LOG_ERROR("Invalid ACK (%1x) in DAP response", el->ack);
			log_dap_cmd("ERR", el);
			retval = ERROR_JTAG_DEVICE_ERROR;
			goto done;
		}
	}

	/*
	 * If we found a stalled transaction and a previous transaction
	 * exists, check if it's a READ access.
	 */
	if (found_wait && el != list_first_entry(&dap->cmd_journal, struct dap_cmd, lh)) {
		prev = list_entry(el->lh.prev, struct dap_cmd, lh);
		if (prev->RnW == DPAP_READ) {
			log_dap_cmd("PND", prev);
			/* search for the next OK transaction, it contains
			 * the result of the previous READ */
			tmp = el;
			list_for_each_entry_from(tmp, &dap->cmd_journal, lh) {
				if (tmp->ack == JTAG_ACK_OK_FAULT) {
					/* recover the read value */
					log_dap_cmd("FND", tmp);
					if (el->invalue != el->invalue_buf) {
						uint32_t invalue = le_to_h_u32(tmp->invalue);
						memcpy(el->invalue, &invalue, sizeof(uint32_t));
					}
					prev = NULL;
					break;
				}
			}

			if (prev != NULL) {
				log_dap_cmd("LST", el);

				/*
				* At this point we're sure that no previous
				* transaction completed and the DAP/AP is still
				* in busy state. We know that the next "OK" scan
				* will return the READ result we need to recover.
				* To complete the READ, we just keep polling RDBUFF
				* until the WAIT condition clears
				*/
				tmp = dap_cmd_new(dap, JTAG_DP_DPACC,
						DP_RDBUFF, DPAP_READ, NULL, NULL, 0);
				if (tmp == NULL) {
					retval = ERROR_JTAG_DEVICE_ERROR;
					goto done;
				}
				/* synchronously retry the command until it succeeds */
				time_now = timeval_ms();
				do {
					retval = adi_jtag_dp_scan_cmd_sync(dap, tmp, NULL);
					if (retval != ERROR_OK)
						break;
					if (tmp->ack == JTAG_ACK_OK_FAULT) {
						log_dap_cmd("FND", tmp);
						if (el->invalue != el->invalue_buf) {
							uint32_t invalue = le_to_h_u32(tmp->invalue);
							memcpy(el->invalue, &invalue, sizeof(uint32_t));
						}
						break;
					}
					if (tmp->ack != JTAG_ACK_WAIT) {
						LOG_ERROR("Invalid ACK (%1x) in DAP response", tmp->ack);
						log_dap_cmd("ERR", tmp);
						retval = ERROR_JTAG_DEVICE_ERROR;
						break;
					}

				} while (timeval_ms() - time_now < 1000);

				if (retval == ERROR_OK) {
					/* timeout happened */
					if (tmp->ack != JTAG_ACK_OK_FAULT) {
						LOG_ERROR("Timeout during WAIT recovery");
						dap->select = DP_SELECT_INVALID;
						jtag_ap_q_abort(dap, NULL);
						/* clear the sticky overrun condition */
						adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
							DP_CTRL_STAT, DPAP_WRITE,
							dap->dp_ctrl_stat | SSTICKYORUN, NULL, 0);
						retval = ERROR_JTAG_DEVICE_ERROR;
					}
				}

				/* we're done with this command, release it */
				dap_cmd_release(dap, tmp);

				if (retval != ERROR_OK)
					goto done;

			}
			/* make el->invalue point to the default invalue
			* so that we can safely retry it without clobbering
			* the result we just recovered */
			el->invalue = el->invalue_buf;
		}
	}

	/* move all remaining transactions over to the replay list */
	list_for_each_entry_safe_from(el, tmp, &dap->cmd_journal, lh) {
		log_dap_cmd("REP", el);
		list_move_tail(&el->lh, &replay_list);
	}

	/* we're done with the journal, flush it */
	flush_journal(dap, &dap->cmd_journal);

	/* check for overrun condition in the last batch of transactions */
	if (found_wait) {
		LOG_INFO("DAP transaction stalled (WAIT) - slowing down");
		/* clear the sticky overrun condition */
		retval = adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
				DP_CTRL_STAT, DPAP_WRITE,
				dap->dp_ctrl_stat | SSTICKYORUN, NULL, 0);
		if (retval != ERROR_OK)
			goto done;

		/* restore SELECT register first */
		if (!list_empty(&replay_list)) {
			el = list_first_entry(&replay_list, struct dap_cmd, lh);
			tmp = dap_cmd_new(dap, JTAG_DP_DPACC,
					  DP_SELECT, DPAP_WRITE, (uint8_t *)&el->dp_select, NULL, 0);
			if (tmp == NULL) {
				retval = ERROR_JTAG_DEVICE_ERROR;
				goto done;
			}
			list_add(&tmp->lh, &replay_list);

			dap->select = DP_SELECT_INVALID;
		}

		list_for_each_entry_safe(el, tmp, &replay_list, lh) {
			time_now = timeval_ms();
			do {
				retval = adi_jtag_dp_scan_cmd_sync(dap, el, NULL);
				if (retval != ERROR_OK)
					break;
				log_dap_cmd("REC", el);
				if (el->ack == JTAG_ACK_OK_FAULT) {
					if (el->invalue != el->invalue_buf) {
						uint32_t invalue = le_to_h_u32(el->invalue);
						memcpy(el->invalue, &invalue, sizeof(uint32_t));
					}
					break;
				}
				if (el->ack != JTAG_ACK_WAIT) {
					LOG_ERROR("Invalid ACK (%1x) in DAP response", el->ack);
					log_dap_cmd("ERR", el);
					retval = ERROR_JTAG_DEVICE_ERROR;
					break;
				}
			} while (timeval_ms() - time_now < 1000);

			if (retval == ERROR_OK) {
				if (el->ack != JTAG_ACK_OK_FAULT) {
					LOG_ERROR("Timeout during WAIT recovery");
					dap->select = DP_SELECT_INVALID;
					jtag_ap_q_abort(dap, NULL);
					/* clear the sticky overrun condition */
					adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
						DP_CTRL_STAT, DPAP_WRITE,
						dap->dp_ctrl_stat | SSTICKYORUN, NULL, 0);
					retval = ERROR_JTAG_DEVICE_ERROR;
					break;
				}
			} else
				break;
		}
	}

 done:
	flush_journal(dap, &replay_list);
	flush_journal(dap, &dap->cmd_journal);
	return retval;
}

static int jtagdp_transaction_endcheck(struct adiv5_dap *dap)
{
	int retval;
	uint32_t ctrlstat, pwrmask;

	/* too expensive to call keep_alive() here */

	/* Post CTRL/STAT read; discard any previous posted read value
	 * but collect its ACK status.
	 */
	retval = adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
			DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat, 0);
	if (retval != ERROR_OK)
		goto done;

	/* REVISIT also STICKYCMP, for pushed comparisons (nyet used) */

	/* Check for STICKYERR */
	if (ctrlstat & SSTICKYERR) {
		LOG_DEBUG("jtag-dp: CTRL/STAT 0x%" PRIx32, ctrlstat);
		/* Check power to debug regions */
		pwrmask = CDBGPWRUPREQ | CDBGPWRUPACK | CSYSPWRUPREQ;
		if (!dap->ignore_syspwrupack)
			pwrmask |= CSYSPWRUPACK;
		if ((ctrlstat & pwrmask) != pwrmask) {
			LOG_ERROR("Debug regions are unpowered, an unexpected reset might have happened");
			dap->do_reconnect = true;
		}

		if (ctrlstat & SSTICKYERR)
			LOG_ERROR("JTAG-DP STICKY ERROR");
		if (ctrlstat & SSTICKYORUN)
			LOG_DEBUG("JTAG-DP STICKY OVERRUN");

		/* Clear Sticky Error Bits */
		retval = adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
				DP_CTRL_STAT, DPAP_WRITE,
				dap->dp_ctrl_stat | SSTICKYERR, NULL, 0);
		if (retval != ERROR_OK)
			goto done;

		retval = ERROR_JTAG_DEVICE_ERROR;
	}

 done:
	flush_journal(dap, &dap->cmd_journal);
	return retval;
}

/*--------------------------------------------------------------------------*/

static int jtag_connect(struct adiv5_dap *dap)
{
	dap->do_reconnect = false;
	return dap_dp_init(dap);
}

static int jtag_check_reconnect(struct adiv5_dap *dap)
{
	if (dap->do_reconnect)
		return jtag_connect(dap);

	return ERROR_OK;
}

static int jtag_send_sequence(struct adiv5_dap *dap, enum swd_special_seq seq)
{
	int retval;

	switch (seq) {
	case JTAG_TO_SWD:
		retval =  jtag_add_tms_seq(swd_seq_jtag_to_swd_len,
				swd_seq_jtag_to_swd, TAP_INVALID);
		break;
	case SWD_TO_JTAG:
		retval = jtag_add_tms_seq(swd_seq_swd_to_jtag_len,
				swd_seq_swd_to_jtag, TAP_RESET);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}
	if (retval == ERROR_OK)
		retval = jtag_execute_queue();
	return retval;
}

static int jtag_dp_q_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	int retval = jtag_limit_queue_size(dap);
	if (retval != ERROR_OK)
		return retval;

	retval =  adi_jtag_dp_scan_u32(dap, JTAG_DP_DPACC, reg,
			DPAP_READ, 0, dap->last_read, 0, NULL);
	dap->last_read = data;
	return retval;
}

static int jtag_dp_q_write(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	int retval = jtag_limit_queue_size(dap);
	if (retval != ERROR_OK)
		return retval;

	retval =  adi_jtag_dp_scan_u32(dap, JTAG_DP_DPACC,
			reg, DPAP_WRITE, data, dap->last_read, 0, NULL);
	dap->last_read = NULL;
	return retval;
}

/** Select the AP register bank matching bits 7:4 of reg. */
static int jtag_ap_q_bankselect(struct adiv5_ap *ap, unsigned reg)
{
	struct adiv5_dap *dap = ap->dap;
	uint32_t sel = ((uint32_t)ap->ap_num << 24) | (reg & 0x000000F0);

	if (sel == dap->select)
		return ERROR_OK;

	dap->select = sel;

	return jtag_dp_q_write(dap, DP_SELECT, sel);
}

static int jtag_ap_q_read(struct adiv5_ap *ap, unsigned reg,
		uint32_t *data)
{
	int retval = jtag_limit_queue_size(ap->dap);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_check_reconnect(ap->dap);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_ap_q_bankselect(ap, reg);
	if (retval != ERROR_OK)
		return retval;

	retval =  adi_jtag_dp_scan_u32(ap->dap, JTAG_DP_APACC, reg,
			DPAP_READ, 0, ap->dap->last_read, ap->memaccess_tck, NULL);
	ap->dap->last_read = data;

	return retval;
}

static int jtag_ap_q_write(struct adiv5_ap *ap, unsigned reg,
		uint32_t data)
{
	int retval = jtag_limit_queue_size(ap->dap);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_check_reconnect(ap->dap);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_ap_q_bankselect(ap, reg);
	if (retval != ERROR_OK)
		return retval;

	retval =  adi_jtag_dp_scan_u32(ap->dap, JTAG_DP_APACC, reg,
			DPAP_WRITE, data, ap->dap->last_read, ap->memaccess_tck, NULL);
	ap->dap->last_read = NULL;
	return retval;
}

static int jtag_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	/* for JTAG, this is the only valid ABORT register operation */
	int retval =  adi_jtag_dp_scan_u32(dap, JTAG_DP_ABORT,
			0, DPAP_WRITE, 1, NULL, 0, NULL);
	if (retval != ERROR_OK)
		return retval;

	return jtag_execute_queue();
}

static int jtag_dp_run(struct adiv5_dap *dap)
{
	int retval;
	int retval2 = ERROR_OK;

	retval = adi_jtag_finish_read(dap);
	if (retval != ERROR_OK)
		goto done;
	retval2 = jtagdp_overrun_check(dap);
	retval = jtagdp_transaction_endcheck(dap);

 done:
	return (retval2 != ERROR_OK) ? retval2 : retval;
}

static int jtag_dp_sync(struct adiv5_dap *dap)
{
	return jtagdp_overrun_check(dap);
}

/* FIXME don't export ... just initialize as
 * part of DAP setup
*/
const struct dap_ops jtag_dp_ops = {
	.connect             = jtag_connect,
	.send_sequence       = jtag_send_sequence,
	.queue_dp_read       = jtag_dp_q_read,
	.queue_dp_write      = jtag_dp_q_write,
	.queue_ap_read       = jtag_ap_q_read,
	.queue_ap_write      = jtag_ap_q_write,
	.queue_ap_abort      = jtag_ap_q_abort,
	.run                 = jtag_dp_run,
	.sync                = jtag_dp_sync,
	.quit                = jtag_quit,
};
