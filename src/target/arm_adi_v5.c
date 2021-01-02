/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009-2010 by Oyvind Harboe                              *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009-2010 by David Brownell                             *
 *                                                                         *
 *   Copyright (C) 2013 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
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

/**
 * @file
 * This file implements support for the ARM Debug Interface version 5 (ADIv5)
 * debugging architecture.  Compared with previous versions, this includes
 * a low pin-count Serial Wire Debug (SWD) alternative to JTAG for message
 * transport, and focusses on memory mapped resources as defined by the
 * CoreSight architecture.
 *
 * A key concept in ADIv5 is the Debug Access Port, or DAP.  A DAP has two
 * basic components:  a Debug Port (DP) transporting messages to and from a
 * debugger, and an Access Port (AP) accessing resources.  Three types of DP
 * are defined.  One uses only JTAG for communication, and is called JTAG-DP.
 * One uses only SWD for communication, and is called SW-DP.  The third can
 * use either SWD or JTAG, and is called SWJ-DP.  The most common type of AP
 * is used to access memory mapped resources and is called a MEM-AP.  Also a
 * JTAG-AP is also defined, bridging to JTAG resources; those are uncommon.
 *
 * This programming interface allows DAP pipelined operations through a
 * transaction queue.  This primarily affects AP operations (such as using
 * a MEM-AP to access memory or registers).  If the current transaction has
 * not finished by the time the next one must begin, and the ORUNDETECT bit
 * is set in the DP_CTRL_STAT register, the SSTICKYORUN status is set and
 * further AP operations will fail.  There are two basic methods to avoid
 * such overrun errors.  One involves polling for status instead of using
 * transaction pipelining.  The other involves adding delays to ensure the
 * AP has enough time to complete one operation before starting the next
 * one.  (For JTAG these delays are controlled by memaccess_tck.)
 */

/*
 * Relevant specifications from ARM include:
 *
 * ARM(tm) Debug Interface v5 Architecture Specification    ARM IHI 0031E
 * CoreSight(tm) v1.0 Architecture Specification            ARM IHI 0029B
 *
 * CoreSight(tm) DAP-Lite TRM, ARM DDI 0316D
 * Cortex-M3(tm) TRM, ARM DDI 0337G
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/interface.h"
#include "arm.h"
#include "arm_adi_v5.h"
#include "jtag/swd.h"
#include "transport/transport.h"
#include <helper/jep106.h>
#include <helper/time_support.h>
#include <helper/list.h>
#include <helper/jim-nvp.h>

/* ARM ADI Specification requires at least 10 bits used for TAR autoincrement  */

/*
	uint32_t tar_block_size(uint32_t address)
	Return the largest block starting at address that does not cross a tar block size alignment boundary
*/
static uint32_t max_tar_block_size(uint32_t tar_autoincr_block, uint32_t address)
{
	return tar_autoincr_block - ((tar_autoincr_block - 1) & address);
}

/***************************************************************************
 *                                                                         *
 * DP and MEM-AP  register access  through APACC and DPACC                 *
 *                                                                         *
***************************************************************************/

static int mem_ap_setup_csw(struct adiv5_ap *ap, uint32_t csw)
{
	csw |= ap->csw_default;

	if (csw != ap->csw_value) {
		/* LOG_DEBUG("DAP: Set CSW %x",csw); */
		int retval = dap_queue_ap_write(ap, MEM_AP_REG_CSW, csw);
		if (retval != ERROR_OK) {
			ap->csw_value = 0;
			return retval;
		}
		ap->csw_value = csw;
	}
	return ERROR_OK;
}

static int mem_ap_setup_tar(struct adiv5_ap *ap, uint32_t tar)
{
	if (!ap->tar_valid || tar != ap->tar_value) {
		/* LOG_DEBUG("DAP: Set TAR %x",tar); */
		int retval = dap_queue_ap_write(ap, MEM_AP_REG_TAR, tar);
		if (retval != ERROR_OK) {
			ap->tar_valid = false;
			return retval;
		}
		ap->tar_value = tar;
		ap->tar_valid = true;
	}
	return ERROR_OK;
}

static int mem_ap_read_tar(struct adiv5_ap *ap, uint32_t *tar)
{
	int retval = dap_queue_ap_read(ap, MEM_AP_REG_TAR, tar);
	if (retval != ERROR_OK) {
		ap->tar_valid = false;
		return retval;
	}

	retval = dap_run(ap->dap);
	if (retval != ERROR_OK) {
		ap->tar_valid = false;
		return retval;
	}

	ap->tar_value = *tar;
	ap->tar_valid = true;
	return ERROR_OK;
}

static uint32_t mem_ap_get_tar_increment(struct adiv5_ap *ap)
{
	switch (ap->csw_value & CSW_ADDRINC_MASK) {
	case CSW_ADDRINC_SINGLE:
		switch (ap->csw_value & CSW_SIZE_MASK) {
		case CSW_8BIT:
			return 1;
		case CSW_16BIT:
			return 2;
		case CSW_32BIT:
			return 4;
		default:
			return 0;
		}
	case CSW_ADDRINC_PACKED:
		return 4;
	}
	return 0;
}

/* mem_ap_update_tar_cache is called after an access to MEM_AP_REG_DRW
 */
static void mem_ap_update_tar_cache(struct adiv5_ap *ap)
{
	if (!ap->tar_valid)
		return;

	uint32_t inc = mem_ap_get_tar_increment(ap);
	if (inc >= max_tar_block_size(ap->tar_autoincr_block, ap->tar_value))
		ap->tar_valid = false;
	else
		ap->tar_value += inc;
}

/**
 * Queue transactions setting up transfer parameters for the
 * currently selected MEM-AP.
 *
 * Subsequent transfers using registers like MEM_AP_REG_DRW or MEM_AP_REG_BD2
 * initiate data reads or writes using memory or peripheral addresses.
 * If the CSW is configured for it, the TAR may be automatically
 * incremented after each transfer.
 *
 * @param ap The MEM-AP.
 * @param csw MEM-AP Control/Status Word (CSW) register to assign.  If this
 *	matches the cached value, the register is not changed.
 * @param tar MEM-AP Transfer Address Register (TAR) to assign.  If this
 *	matches the cached address, the register is not changed.
 *
 * @return ERROR_OK if the transaction was properly queued, else a fault code.
 */
static int mem_ap_setup_transfer(struct adiv5_ap *ap, uint32_t csw, uint32_t tar)
{
	int retval;
	retval = mem_ap_setup_csw(ap, csw);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_setup_tar(ap, tar);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

/**
 * Asynchronous (queued) read of a word from memory or a system register.
 *
 * @param ap The MEM-AP to access.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the word will be stored when the
 *	transaction queue is flushed (assuming no errors).
 *
 * @return ERROR_OK for success.  Otherwise a fault code.
 */
int mem_ap_read_u32(struct adiv5_ap *ap, uint32_t address,
		uint32_t *value)
{
	int retval;

	/* Use banked addressing (REG_BDx) to avoid some link traffic
	 * (updating TAR) when reading several consecutive addresses.
	 */
	retval = mem_ap_setup_transfer(ap,
			CSW_32BIT | (ap->csw_value & CSW_ADDRINC_MASK),
			address & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;

	return dap_queue_ap_read(ap, MEM_AP_REG_BD0 | (address & 0xC), value);
}

/**
 * Synchronous read of a word from memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param ap The MEM-AP to access.
 * @param address Address of the 32-bit word to read; it must be
 *	readable by the currently selected MEM-AP.
 * @param value points to where the result will be stored.
 *
 * @return ERROR_OK for success; *value holds the result.
 * Otherwise a fault code.
 */
int mem_ap_read_atomic_u32(struct adiv5_ap *ap, uint32_t address,
		uint32_t *value)
{
	int retval;

	retval = mem_ap_read_u32(ap, address, value);
	if (retval != ERROR_OK)
		return retval;

	return dap_run(ap->dap);
}

/**
 * Asynchronous (queued) write of a word to memory or a system register.
 *
 * @param ap The MEM-AP to access.
 * @param address Address to be written; it must be writable by
 *	the currently selected MEM-AP.
 * @param value Word that will be written to the address when transaction
 *	queue is flushed (assuming no errors).
 *
 * @return ERROR_OK for success.  Otherwise a fault code.
 */
int mem_ap_write_u32(struct adiv5_ap *ap, uint32_t address,
		uint32_t value)
{
	int retval;

	/* Use banked addressing (REG_BDx) to avoid some link traffic
	 * (updating TAR) when writing several consecutive addresses.
	 */
	retval = mem_ap_setup_transfer(ap,
			CSW_32BIT | (ap->csw_value & CSW_ADDRINC_MASK),
			address & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;

	return dap_queue_ap_write(ap, MEM_AP_REG_BD0 | (address & 0xC),
			value);
}

/**
 * Synchronous write of a word to memory or a system register.
 * As a side effect, this flushes any queued transactions.
 *
 * @param ap The MEM-AP to access.
 * @param address Address to be written; it must be writable by
 *	the currently selected MEM-AP.
 * @param value Word that will be written.
 *
 * @return ERROR_OK for success; the data was written.  Otherwise a fault code.
 */
int mem_ap_write_atomic_u32(struct adiv5_ap *ap, uint32_t address,
		uint32_t value)
{
	int retval = mem_ap_write_u32(ap, address, value);

	if (retval != ERROR_OK)
		return retval;

	return dap_run(ap->dap);
}

/**
 * Synchronous write of a block of memory, using a specific access size.
 *
 * @param ap The MEM-AP to access.
 * @param buffer The data buffer to write. No particular alignment is assumed.
 * @param size Which access size to use, in bytes. 1, 2 or 4.
 * @param count The number of writes to do (in size units, not bytes).
 * @param address Address to be written; it must be writable by the currently selected MEM-AP.
 * @param addrinc Whether the target address should be increased for each write or not. This
 *  should normally be true, except when writing to e.g. a FIFO.
 * @return ERROR_OK on success, otherwise an error code.
 */
static int mem_ap_write(struct adiv5_ap *ap, const uint8_t *buffer, uint32_t size, uint32_t count,
		uint32_t address, bool addrinc)
{
	struct adiv5_dap *dap = ap->dap;
	size_t nbytes = size * count;
	const uint32_t csw_addrincr = addrinc ? CSW_ADDRINC_SINGLE : CSW_ADDRINC_OFF;
	uint32_t csw_size;
	uint32_t addr_xor;
	int retval = ERROR_OK;

	/* TI BE-32 Quirks mode:
	 * Writes on big-endian TMS570 behave very strangely. Observed behavior:
	 *   size   write address   bytes written in order
	 *   4      TAR ^ 0         (val >> 24), (val >> 16), (val >> 8), (val)
	 *   2      TAR ^ 2         (val >> 8), (val)
	 *   1      TAR ^ 3         (val)
	 * For example, if you attempt to write a single byte to address 0, the processor
	 * will actually write a byte to address 3.
	 *
	 * To make writes of size < 4 work as expected, we xor a value with the address before
	 * setting the TAP, and we set the TAP after every transfer rather then relying on
	 * address increment. */

	if (size == 4) {
		csw_size = CSW_32BIT;
		addr_xor = 0;
	} else if (size == 2) {
		csw_size = CSW_16BIT;
		addr_xor = dap->ti_be_32_quirks ? 2 : 0;
	} else if (size == 1) {
		csw_size = CSW_8BIT;
		addr_xor = dap->ti_be_32_quirks ? 3 : 0;
	} else {
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	if (ap->unaligned_access_bad && (address % size != 0))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	while (nbytes > 0) {
		uint32_t this_size = size;

		/* Select packed transfer if possible */
		if (addrinc && ap->packed_transfers && nbytes >= 4
				&& max_tar_block_size(ap->tar_autoincr_block, address) >= 4) {
			this_size = 4;
			retval = mem_ap_setup_csw(ap, csw_size | CSW_ADDRINC_PACKED);
		} else {
			retval = mem_ap_setup_csw(ap, csw_size | csw_addrincr);
		}

		if (retval != ERROR_OK)
			break;

		retval = mem_ap_setup_tar(ap, address ^ addr_xor);
		if (retval != ERROR_OK)
			return retval;

		/* How many source bytes each transfer will consume, and their location in the DRW,
		 * depends on the type of transfer and alignment. See ARM document IHI0031C. */
		uint32_t outvalue = 0;
		uint32_t drw_byte_idx = address;
		if (dap->ti_be_32_quirks) {
			switch (this_size) {
			case 4:
				outvalue |= (uint32_t)*buffer++ << 8 * (3 ^ (drw_byte_idx++ & 3) ^ addr_xor);
				outvalue |= (uint32_t)*buffer++ << 8 * (3 ^ (drw_byte_idx++ & 3) ^ addr_xor);
				outvalue |= (uint32_t)*buffer++ << 8 * (3 ^ (drw_byte_idx++ & 3) ^ addr_xor);
				outvalue |= (uint32_t)*buffer++ << 8 * (3 ^ (drw_byte_idx & 3) ^ addr_xor);
				break;
			case 2:
				outvalue |= (uint32_t)*buffer++ << 8 * (1 ^ (drw_byte_idx++ & 3) ^ addr_xor);
				outvalue |= (uint32_t)*buffer++ << 8 * (1 ^ (drw_byte_idx & 3) ^ addr_xor);
				break;
			case 1:
				outvalue |= (uint32_t)*buffer++ << 8 * (0 ^ (drw_byte_idx & 3) ^ addr_xor);
				break;
			}
		} else {
			switch (this_size) {
			case 4:
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				/* fallthrough */
			case 2:
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx++ & 3);
				/* fallthrough */
			case 1:
				outvalue |= (uint32_t)*buffer++ << 8 * (drw_byte_idx & 3);
			}
		}

		nbytes -= this_size;

		retval = dap_queue_ap_write(ap, MEM_AP_REG_DRW, outvalue);
		if (retval != ERROR_OK)
			break;

		mem_ap_update_tar_cache(ap);
		if (addrinc)
			address += this_size;
	}

	/* REVISIT: Might want to have a queued version of this function that does not run. */
	if (retval == ERROR_OK)
		retval = dap_run(dap);

	if (retval != ERROR_OK) {
		uint32_t tar;
		if (mem_ap_read_tar(ap, &tar) == ERROR_OK)
			LOG_ERROR("Failed to write memory at 0x%08"PRIx32, tar);
		else
			LOG_ERROR("Failed to write memory and, additionally, failed to find out where");
	}

	return retval;
}

/**
 * Synchronous read of a block of memory, using a specific access size.
 *
 * @param ap The MEM-AP to access.
 * @param buffer The data buffer to receive the data. No particular alignment is assumed.
 * @param size Which access size to use, in bytes. 1, 2 or 4.
 * @param count The number of reads to do (in size units, not bytes).
 * @param adr Address to be read; it must be readable by the currently selected MEM-AP.
 * @param addrinc Whether the target address should be increased after each read or not. This
 *  should normally be true, except when reading from e.g. a FIFO.
 * @return ERROR_OK on success, otherwise an error code.
 */
static int mem_ap_read(struct adiv5_ap *ap, uint8_t *buffer, uint32_t size, uint32_t count,
		uint32_t adr, bool addrinc)
{
	struct adiv5_dap *dap = ap->dap;
	size_t nbytes = size * count;
	const uint32_t csw_addrincr = addrinc ? CSW_ADDRINC_SINGLE : CSW_ADDRINC_OFF;
	uint32_t csw_size;
	uint32_t address = adr;
	int retval = ERROR_OK;

	/* TI BE-32 Quirks mode:
	 * Reads on big-endian TMS570 behave strangely differently than writes.
	 * They read from the physical address requested, but with DRW byte-reversed.
	 * For example, a byte read from address 0 will place the result in the high bytes of DRW.
	 * Also, packed 8-bit and 16-bit transfers seem to sometimes return garbage in some bytes,
	 * so avoid them. */

	if (size == 4)
		csw_size = CSW_32BIT;
	else if (size == 2)
		csw_size = CSW_16BIT;
	else if (size == 1)
		csw_size = CSW_8BIT;
	else
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (ap->unaligned_access_bad && (adr % size != 0))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Allocate buffer to hold the sequence of DRW reads that will be made. This is a significant
	 * over-allocation if packed transfers are going to be used, but determining the real need at
	 * this point would be messy. */
	uint32_t *read_buf = calloc(count, sizeof(uint32_t));
	/* Multiplication count * sizeof(uint32_t) may overflow, calloc() is safe */
	uint32_t *read_ptr = read_buf;
	if (read_buf == NULL) {
		LOG_ERROR("Failed to allocate read buffer");
		return ERROR_FAIL;
	}

	/* Queue up all reads. Each read will store the entire DRW word in the read buffer. How many
	 * useful bytes it contains, and their location in the word, depends on the type of transfer
	 * and alignment. */
	while (nbytes > 0) {
		uint32_t this_size = size;

		/* Select packed transfer if possible */
		if (addrinc && ap->packed_transfers && nbytes >= 4
				&& max_tar_block_size(ap->tar_autoincr_block, address) >= 4) {
			this_size = 4;
			retval = mem_ap_setup_csw(ap, csw_size | CSW_ADDRINC_PACKED);
		} else {
			retval = mem_ap_setup_csw(ap, csw_size | csw_addrincr);
		}
		if (retval != ERROR_OK)
			break;

		retval = mem_ap_setup_tar(ap, address);
		if (retval != ERROR_OK)
			break;

		retval = dap_queue_ap_read(ap, MEM_AP_REG_DRW, read_ptr++);
		if (retval != ERROR_OK)
			break;

		nbytes -= this_size;
		if (addrinc)
			address += this_size;

		mem_ap_update_tar_cache(ap);
	}

	if (retval == ERROR_OK)
		retval = dap_run(dap);

	/* Restore state */
	address = adr;
	nbytes = size * count;
	read_ptr = read_buf;

	/* If something failed, read TAR to find out how much data was successfully read, so we can
	 * at least give the caller what we have. */
	if (retval != ERROR_OK) {
		uint32_t tar;
		if (mem_ap_read_tar(ap, &tar) == ERROR_OK) {
			/* TAR is incremented after failed transfer on some devices (eg Cortex-M4) */
			LOG_ERROR("Failed to read memory at 0x%08"PRIx32, tar);
			if (nbytes > tar - address)
				nbytes = tar - address;
		} else {
			LOG_ERROR("Failed to read memory and, additionally, failed to find out where");
			nbytes = 0;
		}
	}

	/* Replay loop to populate caller's buffer from the correct word and byte lane */
	while (nbytes > 0) {
		uint32_t this_size = size;

		if (addrinc && ap->packed_transfers && nbytes >= 4
				&& max_tar_block_size(ap->tar_autoincr_block, address) >= 4) {
			this_size = 4;
		}

		if (dap->ti_be_32_quirks) {
			switch (this_size) {
			case 4:
				*buffer++ = *read_ptr >> 8 * (3 - (address++ & 3));
				*buffer++ = *read_ptr >> 8 * (3 - (address++ & 3));
				/* fallthrough */
			case 2:
				*buffer++ = *read_ptr >> 8 * (3 - (address++ & 3));
				/* fallthrough */
			case 1:
				*buffer++ = *read_ptr >> 8 * (3 - (address++ & 3));
			}
		} else {
			switch (this_size) {
			case 4:
				*buffer++ = *read_ptr >> 8 * (address++ & 3);
				*buffer++ = *read_ptr >> 8 * (address++ & 3);
				/* fallthrough */
			case 2:
				*buffer++ = *read_ptr >> 8 * (address++ & 3);
				/* fallthrough */
			case 1:
				*buffer++ = *read_ptr >> 8 * (address++ & 3);
			}
		}

		read_ptr++;
		nbytes -= this_size;
	}

	free(read_buf);
	return retval;
}

int mem_ap_read_buf(struct adiv5_ap *ap,
		uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address)
{
	return mem_ap_read(ap, buffer, size, count, address, true);
}

int mem_ap_write_buf(struct adiv5_ap *ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address)
{
	return mem_ap_write(ap, buffer, size, count, address, true);
}

int mem_ap_read_buf_noincr(struct adiv5_ap *ap,
		uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address)
{
	return mem_ap_read(ap, buffer, size, count, address, false);
}

int mem_ap_write_buf_noincr(struct adiv5_ap *ap,
		const uint8_t *buffer, uint32_t size, uint32_t count, uint32_t address)
{
	return mem_ap_write(ap, buffer, size, count, address, false);
}

/*--------------------------------------------------------------------------*/


#define DAP_POWER_DOMAIN_TIMEOUT (10)

/*--------------------------------------------------------------------------*/

/**
 * Invalidate cached DP select and cached TAR and CSW of all APs
 */
void dap_invalidate_cache(struct adiv5_dap *dap)
{
	dap->select = DP_SELECT_INVALID;
	dap->last_read = NULL;

	int i;
	for (i = 0; i <= 255; i++) {
		/* force csw and tar write on the next mem-ap access */
		dap->ap[i].tar_valid = false;
		dap->ap[i].csw_value = 0;
	}
}

/**
 * Initialize a DAP.  This sets up the power domains, prepares the DP
 * for further use and activates overrun checking.
 *
 * @param dap The DAP being initialized.
 */
int dap_dp_init(struct adiv5_dap *dap)
{
	int retval;

	LOG_DEBUG("%s", adiv5_dap_name(dap));

	dap->do_reconnect = false;
	dap_invalidate_cache(dap);

	/*
	 * Early initialize dap->dp_ctrl_stat.
	 * In jtag mode only, if the following queue run (in dap_dp_poll_register)
	 * fails and sets the sticky error, it will trigger the clearing
	 * of the sticky. Without this initialization system and debug power
	 * would be disabled while clearing the sticky error bit.
	 */
	dap->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;

	/*
	 * This write operation clears the sticky error bit in jtag mode only and
	 * is ignored in swd mode. It also powers-up system and debug domains in
	 * both jtag and swd modes, if not done before.
	 */
	retval = dap_queue_dp_write(dap, DP_CTRL_STAT, dap->dp_ctrl_stat | SSTICKYERR);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, NULL);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_dp_write(dap, DP_CTRL_STAT, dap->dp_ctrl_stat);
	if (retval != ERROR_OK)
		return retval;

	/* Check that we have debug power domains activated */
	LOG_DEBUG("DAP: wait CDBGPWRUPACK");
	retval = dap_dp_poll_register(dap, DP_CTRL_STAT,
				      CDBGPWRUPACK, CDBGPWRUPACK,
				      DAP_POWER_DOMAIN_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	if (!dap->ignore_syspwrupack) {
		LOG_DEBUG("DAP: wait CSYSPWRUPACK");
		retval = dap_dp_poll_register(dap, DP_CTRL_STAT,
					      CSYSPWRUPACK, CSYSPWRUPACK,
					      DAP_POWER_DOMAIN_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, NULL);
	if (retval != ERROR_OK)
		return retval;

	/* With debug power on we can activate OVERRUN checking */
	dap->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ | CORUNDETECT;
	retval = dap_queue_dp_write(dap, DP_CTRL_STAT, dap->dp_ctrl_stat);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_dp_read(dap, DP_CTRL_STAT, NULL);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

/**
 * Initialize a DAP or do reconnect if DAP is not accessible.
 *
 * @param dap The DAP being initialized.
 */
int dap_dp_init_or_reconnect(struct adiv5_dap *dap)
{
	LOG_DEBUG("%s", adiv5_dap_name(dap));

	/*
	 * Early initialize dap->dp_ctrl_stat.
	 * In jtag mode only, if the following atomic reads fail and set the
	 * sticky error, it will trigger the clearing of the sticky. Without this
	 * initialization system and debug power would be disabled while clearing
	 * the sticky error bit.
	 */
	dap->dp_ctrl_stat = CDBGPWRUPREQ | CSYSPWRUPREQ;

	dap->do_reconnect = false;

	dap_dp_read_atomic(dap, DP_CTRL_STAT, NULL);
	if (dap->do_reconnect) {
		/* dap connect calls dap_dp_init() after transport dependent initialization */
		return dap->ops->connect(dap);
	} else {
		return dap_dp_init(dap);
	}
}

/**
 * Initialize a DAP.  This sets up the power domains, prepares the DP
 * for further use, and arranges to use AP #0 for all AP operations
 * until dap_ap-select() changes that policy.
 *
 * @param ap The MEM-AP being initialized.
 */
int mem_ap_init(struct adiv5_ap *ap)
{
	/* check that we support packed transfers */
	uint32_t csw, cfg;
	int retval;
	struct adiv5_dap *dap = ap->dap;

	ap->tar_valid = false;
	ap->csw_value = 0;      /* force csw and tar write */
	retval = mem_ap_setup_transfer(ap, CSW_8BIT | CSW_ADDRINC_PACKED, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_ap_read(ap, MEM_AP_REG_CSW, &csw);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_queue_ap_read(ap, MEM_AP_REG_CFG, &cfg);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	if (csw & CSW_ADDRINC_PACKED)
		ap->packed_transfers = true;
	else
		ap->packed_transfers = false;

	/* Packed transfers on TI BE-32 processors do not work correctly in
	 * many cases. */
	if (dap->ti_be_32_quirks)
		ap->packed_transfers = false;

	LOG_DEBUG("MEM_AP Packed Transfers: %s",
			ap->packed_transfers ? "enabled" : "disabled");

	/* The ARM ADI spec leaves implementation-defined whether unaligned
	 * memory accesses work, only work partially, or cause a sticky error.
	 * On TI BE-32 processors, reads seem to return garbage in some bytes
	 * and unaligned writes seem to cause a sticky error.
	 * TODO: it would be nice to have a way to detect whether unaligned
	 * operations are supported on other processors. */
	ap->unaligned_access_bad = dap->ti_be_32_quirks;

	LOG_DEBUG("MEM_AP CFG: large data %d, long address %d, big-endian %d",
			!!(cfg & 0x04), !!(cfg & 0x02), !!(cfg & 0x01));

	return ERROR_OK;
}

/**
 * Put the debug link into SWD mode, if the target supports it.
 * The link's initial mode may be either JTAG (for example,
 * with SWJ-DP after reset) or SWD.
 *
 * Note that targets using the JTAG-DP do not support SWD, and that
 * some targets which could otherwise support it may have been
 * configured to disable SWD signaling
 *
 * @param dap The DAP used
 * @return ERROR_OK or else a fault code.
 */
int dap_to_swd(struct adiv5_dap *dap)
{
	LOG_DEBUG("Enter SWD mode");

	return dap_send_sequence(dap, JTAG_TO_SWD);
}

/**
 * Put the debug link into JTAG mode, if the target supports it.
 * The link's initial mode may be either SWD or JTAG.
 *
 * Note that targets implemented with SW-DP do not support JTAG, and
 * that some targets which could otherwise support it may have been
 * configured to disable JTAG signaling
 *
 * @param dap The DAP used
 * @return ERROR_OK or else a fault code.
 */
int dap_to_jtag(struct adiv5_dap *dap)
{
	LOG_DEBUG("Enter JTAG mode");

	return dap_send_sequence(dap, SWD_TO_JTAG);
}

/* CID interpretation -- see ARM IHI 0029B section 3
 * and ARM IHI 0031A table 13-3.
 */
static const char *class_description[16] = {
	"Reserved", "ROM table", "Reserved", "Reserved",
	"Reserved", "Reserved", "Reserved", "Reserved",
	"Reserved", "CoreSight component", "Reserved", "Peripheral Test Block",
	"Reserved", "OptimoDE DESS",
	"Generic IP component", "PrimeCell or System component"
};

static bool is_dap_cid_ok(uint32_t cid)
{
	return (cid & 0xffff0fff) == 0xb105000d;
}

/*
 * This function checks the ID for each access port to find the requested Access Port type
 */
int dap_find_ap(struct adiv5_dap *dap, enum ap_type type_to_find, struct adiv5_ap **ap_out)
{
	int ap_num;

	/* Maximum AP number is 255 since the SELECT register is 8 bits */
	for (ap_num = 0; ap_num <= DP_APSEL_MAX; ap_num++) {

		/* read the IDR register of the Access Port */
		uint32_t id_val = 0;

		int retval = dap_queue_ap_read(dap_ap(dap, ap_num), AP_REG_IDR, &id_val);
		if (retval != ERROR_OK)
			return retval;

		retval = dap_run(dap);

		/* IDR bits:
		 * 31-28 : Revision
		 * 27-24 : JEDEC bank (0x4 for ARM)
		 * 23-17 : JEDEC code (0x3B for ARM)
		 * 16-13 : Class (0b1000=Mem-AP)
		 * 12-8  : Reserved
		 *  7-4  : AP Variant (non-zero for JTAG-AP)
		 *  3-0  : AP Type (0=JTAG-AP 1=AHB-AP 2=APB-AP 4=AXI-AP)
		 */

		/* Reading register for a non-existent AP should not cause an error,
		 * but just to be sure, try to continue searching if an error does happen.
		 */
		if ((retval == ERROR_OK) &&                  /* Register read success */
			((id_val & IDR_JEP106) == IDR_JEP106_ARM) && /* Jedec codes match */
			((id_val & IDR_TYPE) == type_to_find)) {      /* type matches*/

			LOG_DEBUG("Found %s at AP index: %d (IDR=0x%08" PRIX32 ")",
						(type_to_find == AP_TYPE_AHB3_AP)  ? "AHB3-AP"  :
						(type_to_find == AP_TYPE_AHB5_AP)  ? "AHB5-AP"  :
						(type_to_find == AP_TYPE_APB_AP)  ? "APB-AP"  :
						(type_to_find == AP_TYPE_AXI_AP)  ? "AXI-AP"  :
						(type_to_find == AP_TYPE_JTAG_AP) ? "JTAG-AP" : "Unknown",
						ap_num, id_val);

			*ap_out = &dap->ap[ap_num];
			return ERROR_OK;
		}
	}

	LOG_DEBUG("No %s found",
				(type_to_find == AP_TYPE_AHB3_AP)  ? "AHB3-AP"  :
				(type_to_find == AP_TYPE_AHB5_AP)  ? "AHB5-AP"  :
				(type_to_find == AP_TYPE_APB_AP)  ? "APB-AP"  :
				(type_to_find == AP_TYPE_AXI_AP)  ? "AXI-AP"  :
				(type_to_find == AP_TYPE_JTAG_AP) ? "JTAG-AP" : "Unknown");
	return ERROR_FAIL;
}

int dap_get_debugbase(struct adiv5_ap *ap,
			uint32_t *dbgbase, uint32_t *apid)
{
	struct adiv5_dap *dap = ap->dap;
	int retval;

	retval = dap_queue_ap_read(ap, MEM_AP_REG_BASE, dbgbase);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_ap_read(ap, AP_REG_IDR, apid);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

int dap_lookup_cs_component(struct adiv5_ap *ap,
			uint32_t dbgbase, uint8_t type, uint32_t *addr, int32_t *idx)
{
	uint32_t romentry, entry_offset = 0, component_base, devtype;
	int retval;

	*addr = 0;

	do {
		retval = mem_ap_read_atomic_u32(ap, (dbgbase&0xFFFFF000) |
						entry_offset, &romentry);
		if (retval != ERROR_OK)
			return retval;

		component_base = (dbgbase & 0xFFFFF000)
			+ (romentry & 0xFFFFF000);

		if (romentry & 0x1) {
			uint32_t c_cid1;
			retval = mem_ap_read_atomic_u32(ap, component_base | 0xff4, &c_cid1);
			if (retval != ERROR_OK) {
				LOG_ERROR("Can't read component with base address 0x%" PRIx32
					  ", the corresponding core might be turned off", component_base);
				return retval;
			}
			if (((c_cid1 >> 4) & 0x0f) == 1) {
				retval = dap_lookup_cs_component(ap, component_base,
							type, addr, idx);
				if (retval == ERROR_OK)
					break;
				if (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
					return retval;
			}

			retval = mem_ap_read_atomic_u32(ap,
					(component_base & 0xfffff000) | 0xfcc,
					&devtype);
			if (retval != ERROR_OK)
				return retval;
			if ((devtype & 0xff) == type) {
				if (!*idx) {
					*addr = component_base;
					break;
				} else
					(*idx)--;
			}
		}
		entry_offset += 4;
	} while (romentry > 0);

	if (!*addr)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	return ERROR_OK;
}

static int dap_read_part_id(struct adiv5_ap *ap, uint32_t component_base, uint32_t *cid, uint64_t *pid)
{
	assert((component_base & 0xFFF) == 0);
	assert(ap != NULL && cid != NULL && pid != NULL);

	uint32_t cid0, cid1, cid2, cid3;
	uint32_t pid0, pid1, pid2, pid3, pid4;
	int retval;

	/* IDs are in last 4K section */
	retval = mem_ap_read_u32(ap, component_base + 0xFE0, &pid0);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_read_u32(ap, component_base + 0xFE4, &pid1);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_read_u32(ap, component_base + 0xFE8, &pid2);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_read_u32(ap, component_base + 0xFEC, &pid3);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_read_u32(ap, component_base + 0xFD0, &pid4);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_read_u32(ap, component_base + 0xFF0, &cid0);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_read_u32(ap, component_base + 0xFF4, &cid1);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_read_u32(ap, component_base + 0xFF8, &cid2);
	if (retval != ERROR_OK)
		return retval;
	retval = mem_ap_read_u32(ap, component_base + 0xFFC, &cid3);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_run(ap->dap);
	if (retval != ERROR_OK)
		return retval;

	*cid = (cid3 & 0xff) << 24
			| (cid2 & 0xff) << 16
			| (cid1 & 0xff) << 8
			| (cid0 & 0xff);
	*pid = (uint64_t)(pid4 & 0xff) << 32
			| (pid3 & 0xff) << 24
			| (pid2 & 0xff) << 16
			| (pid1 & 0xff) << 8
			| (pid0 & 0xff);

	return ERROR_OK;
}

/* The designer identity code is encoded as:
 * bits 11:8 : JEP106 Bank (number of continuation codes), only valid when bit 7 is 1.
 * bit 7     : Set when bits 6:0 represent a JEP106 ID and cleared when bits 6:0 represent
 *             a legacy ASCII Identity Code.
 * bits 6:0  : JEP106 Identity Code (without parity) or legacy ASCII code according to bit 7.
 * JEP106 is a standard available from jedec.org
 */

/* Part number interpretations are from Cortex
 * core specs, the CoreSight components TRM
 * (ARM DDI 0314H), CoreSight System Design
 * Guide (ARM DGI 0012D) and ETM specs; also
 * from chip observation (e.g. TI SDTI).
 */

/* The legacy code only used the part number field to identify CoreSight peripherals.
 * This meant that the same part number from two different manufacturers looked the same.
 * It is desirable for all future additions to identify with both part number and JEP106.
 * "ANY_ID" is a wildcard (any JEP106) only to preserve legacy behavior for legacy entries.
 */

#define ANY_ID 0x1000

#define ARM_ID 0x4BB

static const struct {
	uint16_t designer_id;
	uint16_t part_num;
	const char *type;
	const char *full;
} dap_partnums[] = {
	{ ARM_ID, 0x000, "Cortex-M3 SCS",              "(System Control Space)", },
	{ ARM_ID, 0x001, "Cortex-M3 ITM",              "(Instrumentation Trace Module)", },
	{ ARM_ID, 0x002, "Cortex-M3 DWT",              "(Data Watchpoint and Trace)", },
	{ ARM_ID, 0x003, "Cortex-M3 FPB",              "(Flash Patch and Breakpoint)", },
	{ ARM_ID, 0x008, "Cortex-M0 SCS",              "(System Control Space)", },
	{ ARM_ID, 0x00a, "Cortex-M0 DWT",              "(Data Watchpoint and Trace)", },
	{ ARM_ID, 0x00b, "Cortex-M0 BPU",              "(Breakpoint Unit)", },
	{ ARM_ID, 0x00c, "Cortex-M4 SCS",              "(System Control Space)", },
	{ ARM_ID, 0x00d, "CoreSight ETM11",            "(Embedded Trace)", },
	{ ARM_ID, 0x00e, "Cortex-M7 FPB",              "(Flash Patch and Breakpoint)", },
	{ ARM_ID, 0x470, "Cortex-M1 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x471, "Cortex-M0 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x490, "Cortex-A15 GIC",             "(Generic Interrupt Controller)", },
	{ ARM_ID, 0x4a1, "Cortex-A53 ROM",             "(v8 Memory Map ROM Table)", },
	{ ARM_ID, 0x4a2, "Cortex-A57 ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4a3, "Cortex-A53 ROM",             "(v7 Memory Map ROM Table)", },
	{ ARM_ID, 0x4a4, "Cortex-A72 ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4a9, "Cortex-A9 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4aa, "Cortex-A35 ROM",             "(v8 Memory Map ROM Table)", },
	{ ARM_ID, 0x4af, "Cortex-A15 ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4b5, "Cortex-R5 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4c0, "Cortex-M0+ ROM",             "(ROM Table)", },
	{ ARM_ID, 0x4c3, "Cortex-M3 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4c4, "Cortex-M4 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4c7, "Cortex-M7 PPB ROM",          "(Private Peripheral Bus ROM Table)", },
	{ ARM_ID, 0x4c8, "Cortex-M7 ROM",              "(ROM Table)", },
	{ ARM_ID, 0x4e0, "Cortex-A35 ROM",             "(v7 Memory Map ROM Table)", },
	{ ARM_ID, 0x906, "CoreSight CTI",              "(Cross Trigger)", },
	{ ARM_ID, 0x907, "CoreSight ETB",              "(Trace Buffer)", },
	{ ARM_ID, 0x908, "CoreSight CSTF",             "(Trace Funnel)", },
	{ ARM_ID, 0x909, "CoreSight ATBR",             "(Advanced Trace Bus Replicator)", },
	{ ARM_ID, 0x910, "CoreSight ETM9",             "(Embedded Trace)", },
	{ ARM_ID, 0x912, "CoreSight TPIU",             "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x913, "CoreSight ITM",              "(Instrumentation Trace Macrocell)", },
	{ ARM_ID, 0x914, "CoreSight SWO",              "(Single Wire Output)", },
	{ ARM_ID, 0x917, "CoreSight HTM",              "(AHB Trace Macrocell)", },
	{ ARM_ID, 0x920, "CoreSight ETM11",            "(Embedded Trace)", },
	{ ARM_ID, 0x921, "Cortex-A8 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x922, "Cortex-A8 CTI",              "(Cross Trigger)", },
	{ ARM_ID, 0x923, "Cortex-M3 TPIU",             "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x924, "Cortex-M3 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x925, "Cortex-M4 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x930, "Cortex-R4 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x931, "Cortex-R5 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x932, "CoreSight MTB-M0+",          "(Micro Trace Buffer)", },
	{ ARM_ID, 0x941, "CoreSight TPIU-Lite",        "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x950, "Cortex-A9 PTM",              "(Program Trace Macrocell)", },
	{ ARM_ID, 0x955, "Cortex-A5 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x95a, "Cortex-A72 ETM",             "(Embedded Trace)", },
	{ ARM_ID, 0x95b, "Cortex-A17 PTM",             "(Program Trace Macrocell)", },
	{ ARM_ID, 0x95d, "Cortex-A53 ETM",             "(Embedded Trace)", },
	{ ARM_ID, 0x95e, "Cortex-A57 ETM",             "(Embedded Trace)", },
	{ ARM_ID, 0x95f, "Cortex-A15 PTM",             "(Program Trace Macrocell)", },
	{ ARM_ID, 0x961, "CoreSight TMC",              "(Trace Memory Controller)", },
	{ ARM_ID, 0x962, "CoreSight STM",              "(System Trace Macrocell)", },
	{ ARM_ID, 0x975, "Cortex-M7 ETM",              "(Embedded Trace)", },
	{ ARM_ID, 0x9a0, "CoreSight PMU",              "(Performance Monitoring Unit)", },
	{ ARM_ID, 0x9a1, "Cortex-M4 TPIU",             "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x9a4, "CoreSight GPR",              "(Granular Power Requester)", },
	{ ARM_ID, 0x9a5, "Cortex-A5 PMU",              "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9a7, "Cortex-A7 PMU",              "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9a8, "Cortex-A53 CTI",             "(Cross Trigger)", },
	{ ARM_ID, 0x9a9, "Cortex-M7 TPIU",             "(Trace Port Interface Unit)", },
	{ ARM_ID, 0x9ae, "Cortex-A17 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9af, "Cortex-A15 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9b7, "Cortex-R7 PMU",              "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9d3, "Cortex-A53 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9d7, "Cortex-A57 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9d8, "Cortex-A72 PMU",             "(Performance Monitor Unit)", },
	{ ARM_ID, 0x9da, "Cortex-A35 PMU/CTI/ETM",     "(Performance Monitor Unit/Cross Trigger/ETM)", },
	{ ARM_ID, 0xc05, "Cortex-A5 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc07, "Cortex-A7 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc08, "Cortex-A8 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc09, "Cortex-A9 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc0e, "Cortex-A17 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xc0f, "Cortex-A15 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xc14, "Cortex-R4 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc15, "Cortex-R5 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xc17, "Cortex-R7 Debug",            "(Debug Unit)", },
	{ ARM_ID, 0xd03, "Cortex-A53 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd04, "Cortex-A35 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd07, "Cortex-A57 Debug",           "(Debug Unit)", },
	{ ARM_ID, 0xd08, "Cortex-A72 Debug",           "(Debug Unit)", },
	{ 0x097,  0x9af, "MSP432 ROM",                 "(ROM Table)" },
	{ 0x09f,  0xcd0, "Atmel CPU with DSU",         "(CPU)" },
	{ 0x0c1,  0x1db, "XMC4500 ROM",                "(ROM Table)" },
	{ 0x0c1,  0x1df, "XMC4700/4800 ROM",           "(ROM Table)" },
	{ 0x0c1,  0x1ed, "XMC1000 ROM",                "(ROM Table)" },
	{ 0x0E5,  0x000, "SHARC+/Blackfin+",           "", },
	{ 0x0F0,  0x440, "Qualcomm QDSS Component v1", "(Qualcomm Designed CoreSight Component v1)", },
	{ 0x3eb,  0x181, "Tegra 186 ROM",              "(ROM Table)", },
	{ 0x3eb,  0x202, "Denver ETM",                 "(Denver Embedded Trace)", },
	{ 0x3eb,  0x211, "Tegra 210 ROM",              "(ROM Table)", },
	{ 0x3eb,  0x302, "Denver Debug",               "(Debug Unit)", },
	{ 0x3eb,  0x402, "Denver PMU",                 "(Performance Monitor Unit)", },
	/* legacy comment: 0x113: what? */
	{ ANY_ID, 0x120, "TI SDTI",                    "(System Debug Trace Interface)", }, /* from OMAP3 memmap */
	{ ANY_ID, 0x343, "TI DAPCTL",                  "", }, /* from OMAP3 memmap */
};

static int dap_rom_display(struct command_invocation *cmd,
				struct adiv5_ap *ap, uint32_t dbgbase, int depth)
{
	int retval;
	uint64_t pid;
	uint32_t cid;
	char tabs[16] = "";

	if (depth > 16) {
		command_print(cmd, "\tTables too deep");
		return ERROR_FAIL;
	}

	if (depth)
		snprintf(tabs, sizeof(tabs), "[L%02d] ", depth);

	uint32_t base_addr = dbgbase & 0xFFFFF000;
	command_print(cmd, "\t\tComponent base address 0x%08" PRIx32, base_addr);

	retval = dap_read_part_id(ap, base_addr, &cid, &pid);
	if (retval != ERROR_OK) {
		command_print(cmd, "\t\tCan't read component, the corresponding core might be turned off");
		return ERROR_OK; /* Don't abort recursion */
	}

	if (!is_dap_cid_ok(cid)) {
		command_print(cmd, "\t\tInvalid CID 0x%08" PRIx32, cid);
		return ERROR_OK; /* Don't abort recursion */
	}

	/* component may take multiple 4K pages */
	uint32_t size = (pid >> 36) & 0xf;
	if (size > 0)
		command_print(cmd, "\t\tStart address 0x%08" PRIx32, (uint32_t)(base_addr - 0x1000 * size));

	command_print(cmd, "\t\tPeripheral ID 0x%010" PRIx64, pid);

	uint8_t class = (cid >> 12) & 0xf;
	uint16_t part_num = pid & 0xfff;
	uint16_t designer_id = ((pid >> 32) & 0xf) << 8 | ((pid >> 12) & 0xff);

	if (designer_id & 0x80) {
		/* JEP106 code */
		command_print(cmd, "\t\tDesigner is 0x%03" PRIx16 ", %s",
				designer_id, jep106_manufacturer(designer_id >> 8, designer_id & 0x7f));
	} else {
		/* Legacy ASCII ID, clear invalid bits */
		designer_id &= 0x7f;
		command_print(cmd, "\t\tDesigner ASCII code 0x%02" PRIx16 ", %s",
				designer_id, designer_id == 0x41 ? "ARM" : "<unknown>");
	}

	/* default values to be overwritten upon finding a match */
	const char *type = "Unrecognized";
	const char *full = "";

	/* search dap_partnums[] array for a match */
	for (unsigned entry = 0; entry < ARRAY_SIZE(dap_partnums); entry++) {

		if ((dap_partnums[entry].designer_id != designer_id) && (dap_partnums[entry].designer_id != ANY_ID))
			continue;

		if (dap_partnums[entry].part_num != part_num)
			continue;

		type = dap_partnums[entry].type;
		full = dap_partnums[entry].full;
		break;
	}

	command_print(cmd, "\t\tPart is 0x%" PRIx16", %s %s", part_num, type, full);
	command_print(cmd, "\t\tComponent class is 0x%" PRIx8 ", %s", class, class_description[class]);

	if (class == 1) { /* ROM Table */
		uint32_t memtype;
		retval = mem_ap_read_atomic_u32(ap, base_addr | 0xFCC, &memtype);
		if (retval != ERROR_OK)
			return retval;

		if (memtype & 0x01)
			command_print(cmd, "\t\tMEMTYPE system memory present on bus");
		else
			command_print(cmd, "\t\tMEMTYPE system memory not present: dedicated debug bus");

		/* Read ROM table entries from base address until we get 0x00000000 or reach the reserved area */
		for (uint16_t entry_offset = 0; entry_offset < 0xF00; entry_offset += 4) {
			uint32_t romentry;
			retval = mem_ap_read_atomic_u32(ap, base_addr | entry_offset, &romentry);
			if (retval != ERROR_OK)
				return retval;
			command_print(cmd, "\t%sROMTABLE[0x%x] = 0x%" PRIx32 "",
					tabs, entry_offset, romentry);
			if (romentry & 0x01) {
				/* Recurse */
				retval = dap_rom_display(cmd, ap, base_addr + (romentry & 0xFFFFF000), depth + 1);
				if (retval != ERROR_OK)
					return retval;
			} else if (romentry != 0) {
				command_print(cmd, "\t\tComponent not present");
			} else {
				command_print(cmd, "\t%s\tEnd of ROM table", tabs);
				break;
			}
		}
	} else if (class == 9) { /* CoreSight component */
		const char *major = "Reserved", *subtype = "Reserved";

		uint32_t devtype;
		retval = mem_ap_read_atomic_u32(ap, base_addr | 0xFCC, &devtype);
		if (retval != ERROR_OK)
			return retval;
		unsigned minor = (devtype >> 4) & 0x0f;
		switch (devtype & 0x0f) {
		case 0:
			major = "Miscellaneous";
			switch (minor) {
			case 0:
				subtype = "other";
				break;
			case 4:
				subtype = "Validation component";
				break;
			}
			break;
		case 1:
			major = "Trace Sink";
			switch (minor) {
			case 0:
				subtype = "other";
				break;
			case 1:
				subtype = "Port";
				break;
			case 2:
				subtype = "Buffer";
				break;
			case 3:
				subtype = "Router";
				break;
			}
			break;
		case 2:
			major = "Trace Link";
			switch (minor) {
			case 0:
				subtype = "other";
				break;
			case 1:
				subtype = "Funnel, router";
				break;
			case 2:
				subtype = "Filter";
				break;
			case 3:
				subtype = "FIFO, buffer";
				break;
			}
			break;
		case 3:
			major = "Trace Source";
			switch (minor) {
			case 0:
				subtype = "other";
				break;
			case 1:
				subtype = "Processor";
				break;
			case 2:
				subtype = "DSP";
				break;
			case 3:
				subtype = "Engine/Coprocessor";
				break;
			case 4:
				subtype = "Bus";
				break;
			case 6:
				subtype = "Software";
				break;
			}
			break;
		case 4:
			major = "Debug Control";
			switch (minor) {
			case 0:
				subtype = "other";
				break;
			case 1:
				subtype = "Trigger Matrix";
				break;
			case 2:
				subtype = "Debug Auth";
				break;
			case 3:
				subtype = "Power Requestor";
				break;
			}
			break;
		case 5:
			major = "Debug Logic";
			switch (minor) {
			case 0:
				subtype = "other";
				break;
			case 1:
				subtype = "Processor";
				break;
			case 2:
				subtype = "DSP";
				break;
			case 3:
				subtype = "Engine/Coprocessor";
				break;
			case 4:
				subtype = "Bus";
				break;
			case 5:
				subtype = "Memory";
				break;
			}
			break;
		case 6:
			major = "Performance Monitor";
			switch (minor) {
			case 0:
				subtype = "other";
				break;
			case 1:
				subtype = "Processor";
				break;
			case 2:
				subtype = "DSP";
				break;
			case 3:
				subtype = "Engine/Coprocessor";
				break;
			case 4:
				subtype = "Bus";
				break;
			case 5:
				subtype = "Memory";
				break;
			}
			break;
		}
		command_print(cmd, "\t\tType is 0x%02" PRIx8 ", %s, %s",
				(uint8_t)(devtype & 0xff),
				major, subtype);
		/* REVISIT also show 0xfc8 DevId */
	}

	return ERROR_OK;
}

int dap_info_command(struct command_invocation *cmd,
		struct adiv5_ap *ap)
{
	int retval;
	uint32_t dbgbase, apid;
	uint8_t mem_ap;

	/* Now we read ROM table ID registers, ref. ARM IHI 0029B sec  */
	retval = dap_get_debugbase(ap, &dbgbase, &apid);
	if (retval != ERROR_OK)
		return retval;

	command_print(cmd, "AP ID register 0x%8.8" PRIx32, apid);
	if (apid == 0) {
		command_print(cmd, "No AP found at this ap 0x%x", ap->ap_num);
		return ERROR_FAIL;
	}

	switch (apid & (IDR_JEP106 | IDR_TYPE)) {
	case IDR_JEP106_ARM | AP_TYPE_JTAG_AP:
		command_print(cmd, "\tType is JTAG-AP");
		break;
	case IDR_JEP106_ARM | AP_TYPE_AHB3_AP:
		command_print(cmd, "\tType is MEM-AP AHB3");
		break;
	case IDR_JEP106_ARM | AP_TYPE_AHB5_AP:
		command_print(cmd, "\tType is MEM-AP AHB5");
		break;
	case IDR_JEP106_ARM | AP_TYPE_APB_AP:
		command_print(cmd, "\tType is MEM-AP APB");
		break;
	case IDR_JEP106_ARM | AP_TYPE_AXI_AP:
		command_print(cmd, "\tType is MEM-AP AXI");
		break;
	default:
		command_print(cmd, "\tUnknown AP type");
		break;
	}

	/* NOTE: a MEM-AP may have a single CoreSight component that's
	 * not a ROM table ... or have no such components at all.
	 */
	mem_ap = (apid & IDR_CLASS) == AP_CLASS_MEM_AP;
	if (mem_ap) {
		command_print(cmd, "MEM-AP BASE 0x%8.8" PRIx32, dbgbase);

		if (dbgbase == 0xFFFFFFFF || (dbgbase & 0x3) == 0x2) {
			command_print(cmd, "\tNo ROM table present");
		} else {
			if (dbgbase & 0x01)
				command_print(cmd, "\tValid ROM table present");
			else
				command_print(cmd, "\tROM table in legacy format");

			dap_rom_display(cmd, ap, dbgbase & 0xFFFFF000, 0);
		}
	}

	return ERROR_OK;
}

enum adiv5_cfg_param {
	CFG_DAP,
	CFG_AP_NUM,
	CFG_BASEADDR,
	CFG_CTIBASE, /* DEPRECATED */
};

static const Jim_Nvp nvp_config_opts[] = {
	{ .name = "-dap",       .value = CFG_DAP },
	{ .name = "-ap-num",    .value = CFG_AP_NUM },
	{ .name = "-baseaddr",  .value = CFG_BASEADDR },
	{ .name = "-ctibase",   .value = CFG_CTIBASE }, /* DEPRECATED */
	{ .name = NULL, .value = -1 }
};

static int adiv5_jim_spot_configure(Jim_GetOptInfo *goi,
		struct adiv5_dap **dap_p, int *ap_num_p, uint32_t *base_p)
{
	if (!goi->argc)
		return JIM_OK;

	Jim_SetEmptyResult(goi->interp);

	Jim_Nvp *n;
	int e = Jim_Nvp_name2value_obj(goi->interp, nvp_config_opts,
				goi->argv[0], &n);
	if (e != JIM_OK)
		return JIM_CONTINUE;

	/* base_p can be NULL, then '-baseaddr' option is treated as unknown */
	if (!base_p && (n->value == CFG_BASEADDR || n->value == CFG_CTIBASE))
		return JIM_CONTINUE;

	e = Jim_GetOpt_Obj(goi, NULL);
	if (e != JIM_OK)
		return e;

	switch (n->value) {
	case CFG_DAP:
		if (goi->isconfigure) {
			Jim_Obj *o_t;
			struct adiv5_dap *dap;
			e = Jim_GetOpt_Obj(goi, &o_t);
			if (e != JIM_OK)
				return e;
			dap = dap_instance_by_jim_obj(goi->interp, o_t);
			if (!dap) {
				Jim_SetResultString(goi->interp, "DAP name invalid!", -1);
				return JIM_ERR;
			}
			if (*dap_p && *dap_p != dap) {
				Jim_SetResultString(goi->interp,
					"DAP assignment cannot be changed!", -1);
				return JIM_ERR;
			}
			*dap_p = dap;
		} else {
			if (goi->argc)
				goto err_no_param;
			if (!*dap_p) {
				Jim_SetResultString(goi->interp, "DAP not configured", -1);
				return JIM_ERR;
			}
			Jim_SetResultString(goi->interp, adiv5_dap_name(*dap_p), -1);
		}
		break;

	case CFG_AP_NUM:
		if (goi->isconfigure) {
			jim_wide ap_num;
			e = Jim_GetOpt_Wide(goi, &ap_num);
			if (e != JIM_OK)
				return e;
			if (ap_num < 0 || ap_num > DP_APSEL_MAX) {
				Jim_SetResultString(goi->interp, "Invalid AP number!", -1);
				return JIM_ERR;
			}
			*ap_num_p = ap_num;
		} else {
			if (goi->argc)
				goto err_no_param;
			if (*ap_num_p == DP_APSEL_INVALID) {
				Jim_SetResultString(goi->interp, "AP number not configured", -1);
				return JIM_ERR;
			}
			Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, *ap_num_p));
		}
		break;

	case CFG_CTIBASE:
		LOG_WARNING("DEPRECATED! use \'-baseaddr' not \'-ctibase\'");
		/* fall through */
	case CFG_BASEADDR:
		if (goi->isconfigure) {
			jim_wide base;
			e = Jim_GetOpt_Wide(goi, &base);
			if (e != JIM_OK)
				return e;
			*base_p = (uint32_t)base;
		} else {
			if (goi->argc)
				goto err_no_param;
			Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, *base_p));
		}
		break;
	};

	return JIM_OK;

err_no_param:
	Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "NO PARAMS");
	return JIM_ERR;
}

int adiv5_jim_configure(struct target *target, Jim_GetOptInfo *goi)
{
	struct adiv5_private_config *pc;
	int e;

	pc = (struct adiv5_private_config *)target->private_config;
	if (pc == NULL) {
		pc = calloc(1, sizeof(struct adiv5_private_config));
		pc->ap_num = DP_APSEL_INVALID;
		target->private_config = pc;
	}

	target->has_dap = true;

	e = adiv5_jim_spot_configure(goi, &pc->dap, &pc->ap_num, NULL);
	if (e != JIM_OK)
		return e;

	if (pc->dap && !target->dap_configured) {
		if (target->tap_configured) {
			pc->dap = NULL;
			Jim_SetResultString(goi->interp,
				"-chain-position and -dap configparams are mutually exclusive!", -1);
			return JIM_ERR;
		}
		target->tap = pc->dap->tap;
		target->dap_configured = true;
	}

	return JIM_OK;
}

int adiv5_verify_config(struct adiv5_private_config *pc)
{
	if (pc == NULL)
		return ERROR_FAIL;

	if (pc->dap == NULL)
		return ERROR_FAIL;

	return ERROR_OK;
}

int adiv5_jim_mem_ap_spot_configure(struct adiv5_mem_ap_spot *cfg,
		Jim_GetOptInfo *goi)
{
	return adiv5_jim_spot_configure(goi, &cfg->dap, &cfg->ap_num, &cfg->base);
}

int adiv5_mem_ap_spot_init(struct adiv5_mem_ap_spot *p)
{
	p->dap = NULL;
	p->ap_num = DP_APSEL_INVALID;
	p->base = 0;
	return ERROR_OK;
}

COMMAND_HANDLER(handle_dap_info_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t apsel;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		if (apsel > DP_APSEL_MAX) {
			command_print(CMD, "Invalid AP number");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return dap_info_command(CMD, &dap->ap[apsel]);
}

COMMAND_HANDLER(dap_baseaddr_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t apsel, baseaddr;
	int retval;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel > DP_APSEL_MAX) {
			command_print(CMD, "Invalid AP number");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* NOTE:  assumes we're talking to a MEM-AP, which
	 * has a base address.  There are other kinds of AP,
	 * though they're not common for now.  This should
	 * use the ID register to verify it's a MEM-AP.
	 */
	retval = dap_queue_ap_read(dap_ap(dap, apsel), MEM_AP_REG_BASE, &baseaddr);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%8.8" PRIx32, baseaddr);

	return retval;
}

COMMAND_HANDLER(dap_memaccess_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t memaccess_tck;

	switch (CMD_ARGC) {
	case 0:
		memaccess_tck = dap->ap[dap->apsel].memaccess_tck;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], memaccess_tck);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	dap->ap[dap->apsel].memaccess_tck = memaccess_tck;

	command_print(CMD, "memory bus access delay set to %" PRIu32 " tck",
			dap->ap[dap->apsel].memaccess_tck);

	return ERROR_OK;
}

COMMAND_HANDLER(dap_apsel_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t apsel;

	switch (CMD_ARGC) {
	case 0:
		command_print(CMD, "%" PRIu32, dap->apsel);
		return ERROR_OK;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel > DP_APSEL_MAX) {
			command_print(CMD, "Invalid AP number");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	dap->apsel = apsel;
	return ERROR_OK;
}

COMMAND_HANDLER(dap_apcsw_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t apcsw = dap->ap[dap->apsel].csw_default;
	uint32_t csw_val, csw_mask;

	switch (CMD_ARGC) {
	case 0:
		command_print(CMD, "ap %" PRIu32 " selected, csw 0x%8.8" PRIx32,
			dap->apsel, apcsw);
		return ERROR_OK;
	case 1:
		if (strcmp(CMD_ARGV[0], "default") == 0)
			csw_val = CSW_AHB_DEFAULT;
		else
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], csw_val);

		if (csw_val & (CSW_SIZE_MASK | CSW_ADDRINC_MASK)) {
			LOG_ERROR("CSW value cannot include 'Size' and 'AddrInc' bit-fields");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		apcsw = csw_val;
		break;
	case 2:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], csw_val);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], csw_mask);
		if (csw_mask & (CSW_SIZE_MASK | CSW_ADDRINC_MASK)) {
			LOG_ERROR("CSW mask cannot include 'Size' and 'AddrInc' bit-fields");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		apcsw = (apcsw & ~csw_mask) | (csw_val & csw_mask);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	dap->ap[dap->apsel].csw_default = apcsw;

	return 0;
}



COMMAND_HANDLER(dap_apid_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t apsel, apid;
	int retval;

	switch (CMD_ARGC) {
	case 0:
		apsel = dap->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel > DP_APSEL_MAX) {
			command_print(CMD, "Invalid AP number");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = dap_queue_ap_read(dap_ap(dap, apsel), AP_REG_IDR, &apid);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_run(dap);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%8.8" PRIx32, apid);

	return retval;
}

COMMAND_HANDLER(dap_apreg_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t apsel, reg, value;
	struct adiv5_ap *ap;
	int retval;

	if (CMD_ARGC < 2 || CMD_ARGC > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
	/* AP address is in bits 31:24 of DP_SELECT */
	if (apsel > DP_APSEL_MAX) {
		command_print(CMD, "Invalid AP number");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	ap = dap_ap(dap, apsel);

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], reg);
	if (reg >= 256 || (reg & 3)) {
		command_print(CMD, "Invalid reg value (should be less than 256 and 4 bytes aligned)");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (CMD_ARGC == 3) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], value);
		switch (reg) {
		case MEM_AP_REG_CSW:
			ap->csw_value = 0;  /* invalid, in case write fails */
			retval = dap_queue_ap_write(ap, reg, value);
			if (retval == ERROR_OK)
				ap->csw_value = value;
			break;
		case MEM_AP_REG_TAR:
			ap->tar_valid = false;  /* invalid, force write */
			retval = mem_ap_setup_tar(ap, value);
			break;
		default:
			retval = dap_queue_ap_write(ap, reg, value);
			break;
		}
	} else {
		retval = dap_queue_ap_read(ap, reg, &value);
	}
	if (retval == ERROR_OK)
		retval = dap_run(dap);

	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC == 2)
		command_print(CMD, "0x%08" PRIx32, value);

	return retval;
}

COMMAND_HANDLER(dap_dpreg_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	uint32_t reg, value;
	int retval;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg);
	if (reg >= 256 || (reg & 3)) {
		command_print(CMD, "Invalid reg value (should be less than 256 and 4 bytes aligned)");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
		retval = dap_queue_dp_write(dap, reg, value);
	} else {
		retval = dap_queue_dp_read(dap, reg, &value);
	}
	if (retval == ERROR_OK)
		retval = dap_run(dap);

	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC == 1)
		command_print(CMD, "0x%08" PRIx32, value);

	return retval;
}

COMMAND_HANDLER(dap_ti_be_32_quirks_command)
{
	struct adiv5_dap *dap = adiv5_get_dap(CMD_DATA);
	return CALL_COMMAND_HANDLER(handle_command_parse_bool, &dap->ti_be_32_quirks,
		"TI BE-32 quirks mode");
}

const struct command_registration dap_instance_commands[] = {
	{
		.name = "info",
		.handler = handle_dap_info_command,
		.mode = COMMAND_EXEC,
		.help = "display ROM table for MEM-AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "apsel",
		.handler = dap_apsel_command,
		.mode = COMMAND_ANY,
		.help = "Set the currently selected AP (default 0) "
			"and display the result",
		.usage = "[ap_num]",
	},
	{
		.name = "apcsw",
		.handler = dap_apcsw_command,
		.mode = COMMAND_ANY,
		.help = "Set CSW default bits",
		.usage = "[value [mask]]",
	},

	{
		.name = "apid",
		.handler = dap_apid_command,
		.mode = COMMAND_EXEC,
		.help = "return ID register from AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "apreg",
		.handler = dap_apreg_command,
		.mode = COMMAND_EXEC,
		.help = "read/write a register from AP "
			"(reg is byte address of a word register, like 0 4 8...)",
		.usage = "ap_num reg [value]",
	},
	{
		.name = "dpreg",
		.handler = dap_dpreg_command,
		.mode = COMMAND_EXEC,
		.help = "read/write a register from DP "
			"(reg is byte address (bank << 4 | reg) of a word register, like 0 4 8...)",
		.usage = "reg [value]",
	},
	{
		.name = "baseaddr",
		.handler = dap_baseaddr_command,
		.mode = COMMAND_EXEC,
		.help = "return debug base address from MEM-AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "memaccess",
		.handler = dap_memaccess_command,
		.mode = COMMAND_EXEC,
		.help = "set/get number of extra tck for MEM-AP memory "
			"bus access [0-255]",
		.usage = "[cycles]",
	},
	{
		.name = "ti_be_32_quirks",
		.handler = dap_ti_be_32_quirks_command,
		.mode = COMMAND_CONFIG,
		.help = "set/get quirks mode for TI TMS450/TMS570 processors",
		.usage = "[enable]",
	},
	COMMAND_REGISTRATION_DONE
};
