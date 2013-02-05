/***************************************************************************
 *   Copyright (C) 2012 by Hsiangkai Wang                                  *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef _AICE_PORT_H_
#define _AICE_PORT_H_

#define ERROR_AICE_DISCONNECT  (-200)
#define ERROR_AICE_TIMEOUT     (-201)

enum aice_target_state_s {
	AICE_DISCONNECT = 0,
	AICE_TARGET_DETACH,
	AICE_TARGET_UNKNOWN,
	AICE_TARGET_RUNNING,
	AICE_TARGET_HALTED,
	AICE_TARGET_RESET,
	AICE_TARGET_DEBUG_RUNNING,
};

enum aice_srst_type_s {
	AICE_SRST = 0x1,
	AICE_RESET_HOLD = 0x8,
};

enum aice_api_s {
	AICE_OPEN = 0x0,
	AICE_CLOSE,
	AICE_RESET,
	AICE_ASSERT_SRST,
	AICE_RUN,
	AICE_HALT,
	AICE_STEP,
	AICE_READ_REG,
	AICE_WRITE_REG,
	AICE_READ_REG_64,
	AICE_WRITE_REG_64,
	AICE_READ_MEM_UNIT,
	AICE_WRITE_MEM_UNIT,
	AICE_READ_MEM_BULK,
	AICE_WRITE_MEM_BULK,
	AICE_READ_DEBUG_REG,
	AICE_WRITE_DEBUG_REG,
	AICE_IDCODE,
	AICE_STATE,
	AICE_SET_JTAG_CLOCK,
	AICE_SELECT_TARGET,
	AICE_MEMORY_ACCESS,
	AICE_MEMORY_MODE,
	AICE_READ_TLB,
	AICE_CACHE_CTL,
};

enum aice_error_s {
	AICE_OK,
	AICE_ACK,
	AICE_ERROR,
};

enum aice_memory_access {
	AICE_MEMORY_ACC_BUS = 0,
	AICE_MEMORY_ACC_CPU,
};

enum aice_memory_mode {
	AICE_MEMORY_MODE_AUTO = 0,
	AICE_MEMORY_MODE_MEM = 1,
	AICE_MEMORY_MODE_ILM = 2,
	AICE_MEMORY_MODE_DLM = 3,
};

enum aice_cache_ctl_type {
	AICE_CACHE_CTL_L1D_INVALALL = 0,
	AICE_CACHE_CTL_L1D_VA_INVAL,
	AICE_CACHE_CTL_L1D_WBALL,
	AICE_CACHE_CTL_L1D_VA_WB,
	AICE_CACHE_CTL_L1I_INVALALL,
	AICE_CACHE_CTL_L1I_VA_INVAL,
};

extern const char *AICE_MEMORY_ACCESS_NAME[];
extern const char *AICE_MEMORY_MODE_NAME[];

struct aice_port_param_s {
	/** */
	char *device_desc;
	/** */
	char *serial;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
	/** */
	char *adapter_name;
};

struct aice_port_s {
	/** */
	struct aice_port_param_s param;
	/** */
	const struct aice_port *port;
	/** */
	uint32_t retry_times;
};

/** */
extern struct aice_port_api_s aice_usb_layout_api;

/** */
struct aice_port_api_s {
	/** */
	int (*open)(struct aice_port_param_s *param);
	/** */
	int (*close)(void);
	/** */
	int (*reset)(void);
	/** */
	int (*assert_srst)(enum aice_srst_type_s srst);
	/** */
	int (*run)(void);
	/** */
	int (*halt)(void);
	/** */
	int (*step)(void);
	/** */
	int (*read_reg)(uint32_t num, uint32_t *val);
	/** */
	int (*write_reg)(uint32_t num, uint32_t val);
	/** */
	int (*read_reg_64)(uint32_t num, uint64_t *val);
	/** */
	int (*write_reg_64)(uint32_t num, uint64_t val);
	/** */
	int (*read_mem_unit)(uint32_t addr, uint32_t size, uint32_t count,
			uint8_t *buffer);
	/** */
	int (*write_mem_unit)(uint32_t addr, uint32_t size, uint32_t count,
			const uint8_t *buffer);
	/** */
	int (*read_mem_bulk)(uint32_t addr, uint32_t length,
			uint8_t *buffer);
	/** */
	int (*write_mem_bulk)(uint32_t addr, uint32_t length,
			const uint8_t *buffer);
	/** */
	int (*read_debug_reg)(uint32_t addr, uint32_t *val);
	/** */
	int (*write_debug_reg)(uint32_t addr, const uint32_t val);

	/** */
	int (*idcode)(uint32_t *idcode, uint8_t *num_of_idcode);
	/** */
	int (*state)(enum aice_target_state_s *state);

	/** */
	int (*set_jtag_clock)(uint32_t a_clock);
	/** */
	int (*select_target)(uint32_t target_id);

	/** */
	int (*memory_access)(enum aice_memory_access a_access);
	/** */
	int (*memory_mode)(enum aice_memory_mode mode);

	/** */
	int (*read_tlb)(uint32_t virtual_address, uint32_t *physical_address);

	/** */
	int (*cache_ctl)(uint32_t subtype, uint32_t address);

	/** */
	int (*set_retry_times)(uint32_t a_retry_times);
};

#define AICE_PORT_UNKNOWN	0
#define AICE_PORT_AICE_USB	1
#define AICE_PORT_AICE_PIPE	2

/** */
struct aice_port {
	/** */
	char *name;
	/** */
	int type;
	/** */
	struct aice_port_api_s *api;
};

/** */
const struct aice_port *aice_port_get_list(void);

#endif

