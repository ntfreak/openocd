/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2018 by Bohdan Tymkiv                                   *
 *   bohdan.tymkiv@cypress.com bohdan200@gmail.com                         *
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

#include <time.h>

#include "imp.h"
#include "target/target.h"
#include "target/cortex_m.h"
#include "target/breakpoints.h"
#include "target/target_type.h"
#include "time_support.h"
#include "target/algorithm.h"

/**************************************************************************************************
 * PSoC6 device definitions
 *************************************************************************************************/
#define MFLASH_SECTOR_SIZE              (256u * 1024u)
#define WFLASH_SECTOR_SIZE              (32u * 1024u)

#define MEM_BASE_MFLASH                 0x10000000u
#define MEM_BASE_WFLASH                 0x14000000u
#define MEM_WFLASH_SIZE                 32768u
#define MEM_BASE_SFLASH                 0x16000000u
#define RAM_STACK_WA_SIZE               2048u
#define PSOC6_SPCIF_GEOMETRY            0x4025F00Cu

#define PROTECTION_UNKNOWN              0x00u
#define PROTECTION_VIRGIN               0x01u
#define PROTECTION_NORMAL               0x02u
#define PROTECTION_SECURE               0x03u
#define PROTECTION_DEAD                 0x04u

#define MEM_BASE_IPC                    0x40230000u
#define IPC_STRUCT_SIZE                 0x20u
#define MEM_IPC(n)                      (MEM_BASE_IPC + (n) * IPC_STRUCT_SIZE)
#define MEM_IPC_ACQUIRE(n)              (MEM_IPC(n) + 0x00u)
#define MEM_IPC_NOTIFY(n)               (MEM_IPC(n) + 0x08u)
#define MEM_IPC_DATA(n)                 (MEM_IPC(n) + 0x0Cu)
#define MEM_IPC_LOCK_STATUS(n)          (MEM_IPC(n) + 0x10u)

#define MEM_BASE_IPC_INTR               0x40231000u
#define IPC_INTR_STRUCT_SIZE            0x20u
#define MEM_IPC_INTR(n)                 (MEM_BASE_IPC_INTR + (n) * IPC_INTR_STRUCT_SIZE)
#define MEM_IPC_INTR_MASK(n)            (MEM_IPC_INTR(n) + 0x08u)
#define IPC_ACQUIRE_SUCCESS_MSK         0x80000000u
#define IPC_LOCK_ACQUIRED_MSK           0x80000000u

#define IPC_ID                          2u
#define IPC_INTR_ID                     0u
#define IPC_TIMEOUT_MS                  1000

#define SROMAPI_SIID_REQ                    0x00000001u
#define SROMAPI_SIID_REQ_FAMILY_REVISION    (SROMAPI_SIID_REQ | 0x000u)
#define SROMAPI_SIID_REQ_SIID_PROTECTION    (SROMAPI_SIID_REQ | 0x100u)
#define SROMAPI_WRITEROW_REQ                0x05000100u
#define SROMAPI_PROGRAMROW_REQ              0x06000100u
#define SROMAPI_ERASESECTOR_REQ             0x14000100u
#define SROMAPI_ERASEALL_REQ                0x0A000100u
#define SROMAPI_ERASEROW_REQ                0x1C000100u

#define SROMAPI_STATUS_MSK                  0xF0000000u
#define SROMAPI_STAT_SUCCESS                0xA0000000u
#define SROMAPI_DATA_LOCATION_MSK           0x00000001u
#define SROMAPI_CALL_TIMEOUT_MS             1500

struct psoc6_target_info {
	uint32_t silicon_id;
	uint8_t protection;
	uint32_t main_flash_sz;
	uint32_t row_sz;
	bool is_probed;
};

struct timeout {
	int64_t start_time;
	long timeout_ms;
};

struct row_region {
	uint32_t addr;
	size_t size;
};

static const struct row_region safe_sflash_regions[] = {
	{0x16000800, 0x800},	/* SFLASH: User Data */
	{0x16001A00, 0x200},	/* SFLASH: NAR */
	{0x16005A00, 0xC00},	/* SFLASH: Public Key */
	{0x16007C00, 0x400},	/* SFLASH: TOC2 */
};

#define SFLASH_NUM_REGIONS ARRAY_SIZE(safe_sflash_regions)

static struct working_area *g_stack_area;
static struct armv7m_algorithm g_armv7m_info;

/** ***********************************************************************************************
 * @brief Initializes `struct timeout` structure with given timeout value
 * @param to pointer to `struct timeout` structure
 * @param timeout_ms timeout, in milliseconds
 *************************************************************************************************/
static void timeout_init(struct timeout *to, long timeout_ms)
{
	to->start_time = timeval_ms();
	to->timeout_ms = timeout_ms;
}

/** ***********************************************************************************************
 * @brief Returns true if given `struct timeout` structure has expired
 * @param to pointer to `struct timeout` structure
 * @return true if timeout expired
 *************************************************************************************************/
static bool timeout_expired(struct timeout *to)
{
	return (timeval_ms() - to->start_time) > to->timeout_ms;
}

/** ***********************************************************************************************
 * @brief Starts pseudo flash algorithm and leaves it running. Function allocates working area for
 * algorithm code and CPU stack, adjusts stack pointer, uploads and starts the algorithm.
 * Algorithm (a basic infinite loop) runs asynchronously while driver performs Flash operations.
 *
 * @param target target for the algorithm
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int sromalgo_prepare(struct target *target)
{
	int hr;

	/* Initialize Vector Table Offset register (in case FW modified it) */
	hr = target_write_u32(target, 0xE000ED08, 0x00000000);
	if (hr != ERROR_OK)
		return hr;

	/* Allocate Working Area for Stack and Flash algorithm */
	hr = target_alloc_working_area(target, RAM_STACK_WA_SIZE, &g_stack_area);
	if (hr != ERROR_OK)
		return hr;

	g_armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	g_armv7m_info.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params;
	init_reg_param(&reg_params, "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params.value, 0, 32, g_stack_area->address + g_stack_area->size);

	/* Write basic infinite loop algorithm to target RAM */
	hr = target_write_u32(target, g_stack_area->address, 0xFEE7FEE7);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	hr = target_start_algorithm(target, 0, NULL, 1, &reg_params, g_stack_area->address,
			0, &g_armv7m_info);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	destroy_reg_param(&reg_params);

	return hr;

destroy_rp_free_wa:
	/* Something went wrong, do some cleanup */
	destroy_reg_param(&reg_params);

	if (g_stack_area) {
		target_free_working_area(target, g_stack_area);
		g_stack_area = NULL;
	}

	return hr;
}

/** ***********************************************************************************************
 * @brief Stops running flash algorithm and releases associated resources.
 * This function is also used for cleanup in case of errors so g_stack_area may be NULL.
 * These cases have to be handled gracefully.
 *
 * @param target current target
 *************************************************************************************************/
static void sromalgo_release(struct target *target)
{
	int hr = ERROR_OK;

	if (g_stack_area) {
		/* Stop flash algorithm if it is running */
		if (target->running_alg) {
			hr = target_halt(target);
			if (hr != ERROR_OK)
				goto exit_free_wa;

			hr = target_wait_algorithm(target, 0, NULL, 0, NULL, 0,
					IPC_TIMEOUT_MS, &g_armv7m_info);
			if (hr != ERROR_OK)
				goto exit_free_wa;
		}

exit_free_wa:
		/* Free Stack/Flash algorithm working area */
		target_free_working_area(target, g_stack_area);
		g_stack_area = NULL;
	}
}

/** ***********************************************************************************************
 * @brief Waits for expected IPC lock status. PSoC6 uses IPC structures for inter-core
 * communication. Same IPCs are used to invoke SROM API. IPC structure must be locked prior to
 * invoking any SROM API. This ensures nothing else in the system will use same IPC thus corrupting
 * our data. Locking is performed by ipc_acquire(), this function ensures that IPC is actually
 * in expected state
 *
 * @param target current target
 * @param ipc_id IPC index to poll. IPC #2 is dedicated for DAP access
 * @param lock_expected expected lock status
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int ipc_poll_lock_stat(struct target *target, uint32_t ipc_id, bool lock_expected)
{
	int hr;
	uint32_t reg_val;

	struct timeout to;
	timeout_init(&to, IPC_TIMEOUT_MS);

	while (!timeout_expired(&to)) {
		/* Process any server requests */
		keep_alive();

		/* Read IPC Lock status */
		hr = target_read_u32(target, MEM_IPC_LOCK_STATUS(ipc_id), &reg_val);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to read IPC Lock Status register");
			return hr;
		}

		bool is_locked = (reg_val & IPC_LOCK_ACQUIRED_MSK) != 0;

		if (lock_expected == is_locked)
			return ERROR_OK;
	}

	if (target->coreid) {
		LOG_WARNING("SROM API calls via CM4 target are supported on single-core PSoC6 devices only. "
			"Please perform all Flash-related operations via CM0+ target on dual-core devices.");
	}

	LOG_ERROR("Timeout polling IPC Lock Status");
	return ERROR_TARGET_TIMEOUT;
}

/** ***********************************************************************************************
 * @brief Acquires IPC structure. PSoC6 uses IPC structures for inter-core communication.
 * Same IPCs are used to invoke SROM API. IPC structure must be locked prior to invoking any SROM API.
 * This ensures nothing else in the system will use same IPC thus corrupting our data.
 * This function locks the IPC.
 *
 * @param target current target
 * @param ipc_id ipc_id IPC index to acquire. IPC #2 is dedicated for DAP access
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int ipc_acquire(struct target *target, char ipc_id)
{
	int hr = ERROR_OK;
	bool is_acquired = false;
	uint32_t reg_val;

	struct timeout to;
	timeout_init(&to, IPC_TIMEOUT_MS);

	while (!timeout_expired(&to)) {
		keep_alive();

		hr = target_write_u32(target, MEM_IPC_ACQUIRE(ipc_id), IPC_ACQUIRE_SUCCESS_MSK);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to write to IPC Acquire register");
			return hr;
		}

		/* Check if data is written on first step */
		hr = target_read_u32(target, MEM_IPC_ACQUIRE(ipc_id), &reg_val);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to read IPC Acquire register");
			return hr;
		}

		is_acquired = (reg_val & IPC_ACQUIRE_SUCCESS_MSK) != 0;
		if (is_acquired) {
			/* If IPC structure is acquired, the lock status should be set */
			hr = ipc_poll_lock_stat(target, ipc_id, true);
			break;
		}
	}

	if (!is_acquired)
		LOG_ERROR("Timeout acquiring IPC structure");

	return hr;
}

/** ***********************************************************************************************
 * @brief Invokes SROM API functions which are responsible for Flash operations
 *
 * @param target current target
 * @param req_and_params request id of the function to invoke
 * @param working_area address of memory buffer in target's memory space for SROM API parameters
 * @param data_out pointer to variable which will be populated with execution status
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int call_sromapi(struct target *target,
	uint32_t req_and_params,
	uint32_t working_area,
	uint32_t *data_out)
{
	int hr;

	bool is_data_in_ram = (req_and_params & SROMAPI_DATA_LOCATION_MSK) == 0;

	hr = ipc_acquire(target, IPC_ID);
	if (hr != ERROR_OK)
		return hr;

	if (is_data_in_ram)
		hr = target_write_u32(target, MEM_IPC_DATA(IPC_ID), working_area);
	else
		hr = target_write_u32(target, MEM_IPC_DATA(IPC_ID), req_and_params);

	if (hr != ERROR_OK)
		return hr;

	/* Enable notification interrupt of IPC_INTR_STRUCT0(CM0+) for IPC_STRUCT2 */
	hr = target_write_u32(target, MEM_IPC_INTR_MASK(IPC_INTR_ID), 1u << (16 + IPC_ID));
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, MEM_IPC_NOTIFY(IPC_ID), 1);
	if (hr != ERROR_OK)
		return hr;

	/* Poll lock status */
	hr = ipc_poll_lock_stat(target, IPC_ID, false);
	if (hr != ERROR_OK)
		return hr;

	/* Poll Data byte */
	if (is_data_in_ram)
		hr = target_read_u32(target, working_area, data_out);
	else
		hr = target_read_u32(target, MEM_IPC_DATA(IPC_ID), data_out);

	if (hr != ERROR_OK) {
		LOG_ERROR("Error reading SROM API Status location");
		return hr;
	}

	bool is_success = (*data_out & SROMAPI_STATUS_MSK) == SROMAPI_STAT_SUCCESS;
	if (!is_success) {
		LOG_ERROR("SROM API execution failed. Status: 0x%08" PRIX32, *data_out);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Retrieves SiliconID and Protection status of the target device
 * @param target current target
 * @param si_id pointer to variable, will be populated with SiliconID
 * @param protection pointer to variable, will be populated with protection status
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int get_silicon_id(struct target *target, uint32_t *si_id, uint8_t *protection)
{
	int hr;
	uint32_t family_rev, siid_prot;

	hr = sromalgo_prepare(target);
	if (hr != ERROR_OK)
		goto exit;

	/* Read FamilyID and Revision */
	hr = call_sromapi(target, SROMAPI_SIID_REQ_FAMILY_REVISION, 0, &family_rev);
	if (hr != ERROR_OK)
		goto exit;

	/* Read SiliconID and Protection */
	hr = call_sromapi(target, SROMAPI_SIID_REQ_SIID_PROTECTION, 0, &siid_prot);
	if (hr != ERROR_OK)
		goto exit;

	*si_id  = (siid_prot & 0x0000FFFF) << 16;
	*si_id |= (family_rev & 0x00FF0000) >> 8;
	*si_id |= (family_rev & 0x000000FF) >> 0;

	*protection = (siid_prot & 0x000F0000) >> 0x10;

exit:
	sromalgo_release(target);
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Translates Protection status to openocd-friendly boolean value
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_protect_check(struct flash_bank *bank)
{
	int is_protected;

	struct psoc6_target_info *psoc6_info = bank->driver_priv;
	int hr = get_silicon_id(bank->target, &psoc6_info->silicon_id, &psoc6_info->protection);
	if (hr != ERROR_OK)
		return hr;

	switch (psoc6_info->protection) {
		case PROTECTION_VIRGIN:
		case PROTECTION_NORMAL:
			is_protected = 0;
			break;

		case PROTECTION_UNKNOWN:
		case PROTECTION_SECURE:
		case PROTECTION_DEAD:
		default:
			is_protected = 1;
			break;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = is_protected;

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Dummy function, Life Cycle transition is not currently supported
 * @return ERROR_OK always
 *************************************************************************************************/
static int psoc6_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	(void)bank;
	(void)set;
	(void)first;
	(void)last;

	LOG_WARNING("Life Cycle transition for PSoC6 is not supported");
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Translates Protection status to string
 * @param protection protection value
 * @return pointer to const string describing protection status
 *************************************************************************************************/
static const char *protection_to_str(uint8_t protection)
{
	switch (protection) {
		case PROTECTION_VIRGIN:
			return "VIRGIN";
		case PROTECTION_NORMAL:
			return "NORMAL";
		case PROTECTION_SECURE:
			return "SECURE";
		case PROTECTION_DEAD:
			return "DEAD";
		case PROTECTION_UNKNOWN:
		default:
			return "UNKNOWN";
	}
}

/** ***********************************************************************************************
 * @brief psoc6_get_info Displays human-readable information about acquired device
 * @param bank current flash bank
 * @param buf pointer to buffer for human-readable text
 * @param buf_size size of the buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct psoc6_target_info *psoc6_info = bank->driver_priv;

	if (psoc6_info->is_probed == false)
		return ERROR_FAIL;

	int hr = get_silicon_id(bank->target, &psoc6_info->silicon_id, &psoc6_info->protection);
	if (hr != ERROR_OK)
		return hr;

	snprintf(buf, buf_size,
		"PSoC6 Silicon ID: 0x%08" PRIX32 "\n"
		"Protection: %s\n"
		"Main Flash size: %" PRIu32 " kB\n"
		"Work Flash size: 32 kB\n",
		psoc6_info->silicon_id,
		protection_to_str(psoc6_info->protection),
		psoc6_info->main_flash_sz / 1024);

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Checks if given flash bank belongs to Supervisory Flash
 * @param bank current flash bank
 * @return true if flash bank belongs to Supervisory Flash
 *************************************************************************************************/
static bool is_sflash_bank(struct flash_bank *bank)
{
	for (size_t i = 0; i < SFLASH_NUM_REGIONS; i++) {
		if (bank->base == safe_sflash_regions[i].addr)
			return true;
	}

	return false;
}

/** ***********************************************************************************************
 * @brief Checks if given flash bank belongs to Work Flash
 * @param bank current flash bank
 * @return true if flash bank belongs to Work Flash
 *************************************************************************************************/
static inline bool is_wflash_bank(struct flash_bank *bank)
{
	return (bank->base == MEM_BASE_WFLASH);
}

/** ***********************************************************************************************
 * @brief Checks if given flash bank belongs to Main Flash
 * @param bank current flash bank
 * @return true if flash bank belongs to Main Flash
 *************************************************************************************************/
static inline bool is_mflash_bank(struct flash_bank *bank)
{
	return (bank->base == MEM_BASE_MFLASH);
}

/** ***********************************************************************************************
 * @brief Probes the device and populates related data structures with target flash geometry data.
 * This is done in non-intrusive way, no SROM API calls are involved so GDB can safely attach to a
 * running target. Function assumes that size of Work Flash is 32kB (true for all current part numbers)
 *
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct psoc6_target_info *psoc6_info = bank->driver_priv;

	int hr = ERROR_OK;

	/* Retrieve data from SPCIF_GEOMETRY */
	uint32_t geom;
	target_read_u32(target, PSOC6_SPCIF_GEOMETRY, &geom);
	uint32_t row_sz_lg2 = (geom & 0xF0) >> 4;
	uint32_t row_sz = (0x01 << row_sz_lg2);
	uint32_t row_cnt = 1 + ((geom & 0x00FFFF00) >> 8);
	uint32_t bank_cnt = 1 + ((geom & 0xFF000000) >> 24);

	/* Calculate size of Main Flash*/
	uint32_t flash_sz_bytes = bank_cnt * row_cnt * row_sz;

	free(bank->sectors);
	bank->sectors = NULL;

	size_t bank_size = 0;

	if (is_mflash_bank(bank))
		bank_size = flash_sz_bytes;
	else if (is_wflash_bank(bank))
		bank_size = MEM_WFLASH_SIZE;
	else if (is_sflash_bank(bank)) {
		for (size_t i = 0; i < SFLASH_NUM_REGIONS; i++) {
			if (safe_sflash_regions[i].addr == bank->base) {
				bank_size = safe_sflash_regions[i].size;
				break;
			}
		}
	}

	if (bank_size == 0) {
		LOG_ERROR("Invalid Flash Bank base address in config file");
		return ERROR_FLASH_BANK_INVALID;
	}

	unsigned int num_sectors = bank_size / row_sz;
	bank->size = bank_size;
	bank->chip_width = 4;
	bank->bus_width = 4;
	bank->erased_value = 0;
	bank->default_padded_value = 0;

	bank->num_sectors = num_sectors;
	bank->sectors = calloc(num_sectors, sizeof(struct flash_sector));
	for (unsigned int i = 0; i < num_sectors; i++) {
		bank->sectors[i].size = row_sz;
		bank->sectors[i].offset = i * row_sz;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	psoc6_info->is_probed = true;
	psoc6_info->main_flash_sz = flash_sz_bytes;
	psoc6_info->row_sz = row_sz;

	return hr;
}

/** ***********************************************************************************************
 * @brief Probes target device only if it hasn't been probed yet
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_auto_probe(struct flash_bank *bank)
{
	struct psoc6_target_info *psoc6_info = bank->driver_priv;
	int hr;

	if (psoc6_info->is_probed)
		hr = ERROR_OK;
	else
		hr = psoc6_probe(bank);

	return hr;
}

/** ***********************************************************************************************
 * @brief Erases single sector (256k) on target device
 * @param bank current flash bank
 * @param wa working area for SROM API parameters
 * @param addr starting address of the sector
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_erase_sector(struct flash_bank *bank, struct working_area *wa, uint32_t addr)
{
	struct target *target = bank->target;

	LOG_DEBUG("Erasing SECTOR @%08" PRIX32, addr);

	int hr = target_write_u32(target, wa->address, SROMAPI_ERASESECTOR_REQ);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, wa->address + 0x04, addr);
	if (hr != ERROR_OK)
		return hr;

	uint32_t data_out;
	hr = call_sromapi(target, SROMAPI_ERASESECTOR_REQ, wa->address, &data_out);
	if (hr != ERROR_OK)
		LOG_ERROR("SECTOR @%08" PRIX32 " not erased!", addr);

	return hr;
}

/** ***********************************************************************************************
 * @brief Erases single row (512b) on target device
 * @param bank current flash bank
 * @param wa working area for SROM API parameters
 * @param addr starting address of the flash row
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_erase_row(struct flash_bank *bank, struct working_area *wa, uint32_t addr)
{
	struct target *target = bank->target;

	LOG_DEBUG("Erasing ROW @%08" PRIX32, addr);

	int hr = target_write_u32(target, wa->address, SROMAPI_ERASEROW_REQ);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, wa->address + 0x04, addr);
	if (hr != ERROR_OK)
		return hr;

	uint32_t data_out;
	hr = call_sromapi(target, SROMAPI_ERASEROW_REQ, wa->address, &data_out);
	if (hr != ERROR_OK)
		LOG_ERROR("ROW @%08" PRIX32 " not erased!", addr);

	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Erase operation. Function will try to use biggest erase block possible to
 * speedup the operation.
 *
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct psoc6_target_info *psoc6_info = bank->driver_priv;
	const uint32_t sector_size = is_wflash_bank(bank) ? WFLASH_SECTOR_SIZE : MFLASH_SECTOR_SIZE;

	int hr;
	struct working_area *wa;

	if (is_sflash_bank(bank)) {
		LOG_INFO("Erase operation on Supervisory Flash is not required, skipping");
		return ERROR_OK;
	}

	hr = sromalgo_prepare(target);
	if (hr != ERROR_OK)
		goto exit;

	hr = target_alloc_working_area(target, psoc6_info->row_sz + 32, &wa);
	if (hr != ERROR_OK)
		goto exit;

	/* Number of rows in single sector */
	const unsigned int rows_in_sector = sector_size / psoc6_info->row_sz;

	while (last >= first) {
		/* Erase Sector if we are on sector boundary and erase size covers whole sector */
		if ((first % rows_in_sector) == 0 &&
			(last - first + 1) >= rows_in_sector) {
			hr = psoc6_erase_sector(bank, wa, bank->base + first * psoc6_info->row_sz);
			if (hr != ERROR_OK)
				goto exit_free_wa;

			for (unsigned int i = first; i < first + rows_in_sector; i++)
				bank->sectors[i].is_erased = 1;

			first += rows_in_sector;
		} else {
			/* Perform Row Erase otherwise */
			hr = psoc6_erase_row(bank, wa, bank->base + first * psoc6_info->row_sz);
			if (hr != ERROR_OK)
				goto exit_free_wa;

			bank->sectors[first].is_erased = 1;
			first += 1;
		}
	}

exit_free_wa:
	target_free_working_area(target, wa);
exit:
	sromalgo_release(target);
	return hr;
}

/** ***********************************************************************************************
 * @brief Programs single Flash Row
 * @param bank current flash bank
 * @param addr address of the flash row
 * @param buffer pointer to the buffer with data
 * @param is_sflash true if current flash bank belongs to Supervisory Flash
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_program_row(struct flash_bank *bank,
	uint32_t addr,
	const uint8_t *buffer,
	bool is_sflash)
{
	struct target *target = bank->target;
	struct psoc6_target_info *psoc6_info = bank->driver_priv;
	struct working_area *wa;
	const uint32_t sromapi_req = is_sflash ? SROMAPI_WRITEROW_REQ : SROMAPI_PROGRAMROW_REQ;
	uint32_t data_out;
	int hr = ERROR_OK;

	LOG_DEBUG("Programming ROW @%08" PRIX32, addr);

	hr = target_alloc_working_area(target, psoc6_info->row_sz + 32, &wa);
	if (hr != ERROR_OK)
		goto exit;

	hr = target_write_u32(target, wa->address, sromapi_req);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	hr = target_write_u32(target,
			wa->address + 0x04,
			0x106);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	hr = target_write_u32(target, wa->address + 0x08, addr);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	hr = target_write_u32(target, wa->address + 0x0C, wa->address + 0x10);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	hr = target_write_buffer(target, wa->address + 0x10, psoc6_info->row_sz, buffer);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	hr = call_sromapi(target, sromapi_req, wa->address, &data_out);

exit_free_wa:
	target_free_working_area(target, wa);

exit:
	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Program operation
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in flash bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_program(struct flash_bank *bank,
	const uint8_t *buffer,
	uint32_t offset,
	uint32_t count)
{
	struct target *target = bank->target;
	struct psoc6_target_info *psoc6_info = bank->driver_priv;
	const bool is_sflash = is_sflash_bank(bank);
	int hr;

	uint8_t page_buf[psoc6_info->row_sz];

	hr = sromalgo_prepare(target);
	if (hr != ERROR_OK)
		goto exit;

	while (count) {
		uint32_t row_offset = offset % psoc6_info->row_sz;
		uint32_t aligned_addr = bank->base + offset - row_offset;
		uint32_t row_bytes = MIN(psoc6_info->row_sz - row_offset, count);

		memset(page_buf, 0, sizeof(page_buf));
		memcpy(&page_buf[row_offset], buffer, row_bytes);

		hr = psoc6_program_row(bank, aligned_addr, page_buf, is_sflash);
		if (hr != ERROR_OK) {
			LOG_ERROR("Failed to program Flash at address 0x%08" PRIX32, aligned_addr);
			goto exit;
		}

		buffer += row_bytes;
		offset += row_bytes;
		count -= row_bytes;
	}

exit:
	sromalgo_release(target);
	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Mass Erase operation
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
COMMAND_HANDLER(psoc6_handle_mass_erase_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int hr = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc6_erase(bank, 0, bank->num_sectors - 1);

	return hr;
}

/** ***********************************************************************************************
 * @brief Simulates broken Vector Catch
 * Function will try to determine entry point of user application. If it succeeds it will set HW
 * breakpoint at that address, issue SW Reset and remove the breakpoint afterwards.
 * In case of CM0, SYSRESETREQ is used. This allows to reset all peripherals. Boot code will
 * reset CM4 anyway, so using SYSRESETREQ is safe here.
 * In case of CM4, VECTRESET is used instead of SYSRESETREQ to not disturb CM0 core.
 *
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int handle_reset_halt(struct target *target)
{
	int hr;
	uint32_t reset_addr;
	bool is_cm0 = (target->coreid == 0);

	/* Halt target device */
	if (target->state != TARGET_HALTED) {
		hr = target_halt(target);
		if (hr != ERROR_OK)
			return hr;

		target_wait_state(target, TARGET_HALTED, IPC_TIMEOUT_MS);
		if (hr != ERROR_OK)
			return hr;
	}

	/* Read Vector Offset register */
	uint32_t vt_base;
	const uint32_t vt_offset_reg = is_cm0 ? 0x402102B0 : 0x402102C0;
	hr = target_read_u32(target, vt_offset_reg, &vt_base);
	if (hr != ERROR_OK)
		return ERROR_OK;

	/* Invalid value means flash is empty */
	vt_base &= 0xFFFFFF00;
	if ((vt_base == 0) || (vt_base == 0xFFFFFF00))
		return ERROR_OK;

	/* Read Reset Vector value*/
	hr = target_read_u32(target, vt_base + 4, &reset_addr);
	if (hr != ERROR_OK)
		return hr;

	/* Invalid value means flash is empty */
	if ((reset_addr == 0) || (reset_addr == 0xFFFFFF00))
		return ERROR_OK;


	/* Set breakpoint at User Application entry point */
	hr = breakpoint_add(target, reset_addr, 2, BKPT_HARD);
	if (hr != ERROR_OK)
		return hr;

	const struct armv7m_common *cm = target_to_armv7m(target);

	/* PSoC6 reboots immediately after issuing SYSRESETREQ / VECTRESET
	 * this disables SWD/JTAG pins momentarily and may break communication
	 * Ignoring return value of mem_ap_write_atomic_u32 seems to be ok here */
	if (is_cm0) {
		/* Reset the CM0 by asserting SYSRESETREQ. This will also reset CM4 */
		LOG_INFO("psoc6.cm0: bkpt @0x%08" PRIX32 ", issuing SYSRESETREQ", reset_addr);
		mem_ap_write_atomic_u32(cm->debug_ap, NVIC_AIRCR,
			AIRCR_VECTKEY | AIRCR_SYSRESETREQ);
	} else {
		LOG_INFO("psoc6.cm4: bkpt @0x%08" PRIX32 ", issuing VECTRESET", reset_addr);
		mem_ap_write_atomic_u32(cm->debug_ap, NVIC_AIRCR,
			AIRCR_VECTKEY | AIRCR_VECTRESET);
	}

	/* Wait 100ms for bootcode and reinitialize DAP */
	usleep(100000);
	dap_dp_init(cm->debug_ap->dap);

	target_wait_state(target, TARGET_HALTED, IPC_TIMEOUT_MS);

	/* Remove the break point */
	breakpoint_remove(target, reset_addr);

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Simulates broken Vector Catch
 * Function will try to determine entry point of user application. If it succeeds it will set HW
 * breakpoint at that address, issue SW Reset and remove the breakpoint afterwards.
 * In case of CM0, SYSRESETREQ is used. This allows to reset all peripherals. Boot code will
 * reset CM4 anyway, so using SYSRESETREQ is safe here.
 * In case of CM4, VECTRESET is used instead of SYSRESETREQ to not disturb CM0 core.
 *
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
COMMAND_HANDLER(psoc6_handle_reset_halt)
{
	if (CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	return handle_reset_halt(target);
}

FLASH_BANK_COMMAND_HANDLER(psoc6_flash_bank_command)
{
	struct psoc6_target_info *psoc6_info;
	int hr = ERROR_OK;

	if (CMD_ARGC < 6)
		hr = ERROR_COMMAND_SYNTAX_ERROR;
	else {
		psoc6_info = calloc(1, sizeof(struct psoc6_target_info));
		psoc6_info->is_probed = false;
		bank->driver_priv = psoc6_info;
	}
	return hr;
}

static const struct command_registration psoc6_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = psoc6_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank",
		.help = "Erases entire Main Flash",
	},
	{
		.name = "reset_halt",
		.handler = psoc6_handle_reset_halt,
		.mode = COMMAND_EXEC,
		.usage = NULL,
		.help = "Tries to simulate broken Vector Catch",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc6_command_handlers[] = {
	{
		.name = "psoc6",
		.mode = COMMAND_ANY,
		.help = "PSoC 6 flash command group",
		.usage = "",
		.chain = psoc6_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver psoc6_flash = {
	.name = "psoc6",
	.commands = psoc6_command_handlers,
	.flash_bank_command = psoc6_flash_bank_command,
	.erase = psoc6_erase,
	.protect = psoc6_protect,
	.write = psoc6_program,
	.read = default_flash_read,
	.probe = psoc6_probe,
	.auto_probe = psoc6_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = psoc6_protect_check,
	.info = psoc6_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
