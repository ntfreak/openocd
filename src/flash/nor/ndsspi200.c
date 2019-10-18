#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include "target/riscv/program.h"
#include "target/algorithm.h"

#define REG_SMU_BASE        0xF0100000
#define CPE_SPIB_BASE       0xF0B00000

unsigned int spib200_base = REG_SMU_BASE, spib200_ctrl = 0;

#define SPIB_REG_DCTRL      (spib200_base+0x20)
#define SPIB_REG_CMD        (spib200_base+0x24)
#define SPIB_REG_DATA       (spib200_base+0x2c)
#define SPIB_REG_CTRL       (spib200_base+0x30)
#define SPIB_REG_FIFOST     (spib200_base+0x34)
#define SPIB_REG_REGTIMING  (spib200_base+0x40)

/*-- Data Control Reg --*/
#define SPIB_DCTRL_CMDEN_MASK       0x40000000
#define SPIB_DCTRL_ADDREN_MASK      0x20000000
#define SPIB_DCTRL_TRAMODE_MASK     0x0f000000
#define SPIB_DCTRL_WCNT_MASK        0x001ff000
#define SPIB_DCTRL_DYCNT_MASK       0x00000600
#define SPIB_DCTRL_RCNT_MASK        0x000001ff
#define SPIB_DCTRL_CMDEN_OFFSET     30
#define SPIB_DCTRL_ADDREN_OFFSET    29
#define SPIB_DCTRL_TRAMODE_OFFSET   24
#define SPIB_DCTRL_WCNT_OFFSET      12
#define SPIB_DCTRL_DYCNT_OFFSET     9
#define SPIB_DCTRL_RCNT_OFFSET      0
/*-- Control Reg --*/
#define SPIB_CTRL_TXFRST_MASK       0x00000004
#define SPIB_CTRL_RXFRST_MASK       0x00000002
/*-- FIFO Status Reg --*/
#define SPIB_FIFOST_TXFFL_MASK      0x00800000
#define SPIB_FIFOST_RXFEM_MASK      0x00004000
#define SPIB_FIFOST_SPIBSY_MASK     0x00000001
/*-- SPIB transfer mode--*/
#define SPIB_TM_WRonly              0x1
#define SPIB_TM_RDonly              0x2
#define SPIB_TM_WR_RD               0x3

#define FLASH_RETRY_TIMES   100

/*-- SPI OP --*/
#define SPIROM_OP_RDID      0x9f   /*-- manufacturer/device ID read --*/
#define SPIROM_OP_RDSR      0x05   /*-- read status register --*/
#define SPIROM_OP_WREN      0x06   /*-- write enable --*/
#define SPIROM_OP_SE        0x20   /*-- sector erase --*/
#define SPIROM_OP_WRSR      0x01   /*-- write status register --*/
#define SPIROM_OP_PP        0x02   /*-- page program --*/

#define SPIROM_ID_MASK      0x00ffffff
/*-- bit mask for status register --*/
#define SPIROM_SR_WIP_MASK  0x01   /*-- write in progress --*/
#define SPIROM_SR_WEL_MASK  0x02   /*-- write enable --*/
#define SPIROM_SR_BP_MASK   0x3C   /*-- BP protection mode --*/

/* SPI ROM cmd */
#define SPIROM_CMD_RDID     0x1
#define SPIROM_CMD_RDST     0x2
#define SPIROM_CMD_WREN     0x3
#define SPIROM_CMD_ERASE    0x5
#define SPIROM_CMD_PROGRAM  0x6
#define SPIROM_CMD_WRSR     0xD

struct ndsspi200_flash_bank {
	int probed;
	struct flash_device *dev;
};

uint8_t algorithm_bin[] = {
#include "../../../contrib/loaders/flash/ndsspi/ndsspi200.inc"
};

#define STEP_EXIT			0x4
#define STEP_ERASE			0x8
#define STEP_TX				0xc
#define STEP_NOP			0xff

uint32_t erase_used = 5;
uint32_t tx_used = 5;

struct algorithm_steps {
	uint32_t size;
	uint32_t used;
	uint8_t **steps;
};

static int inw(struct flash_bank *bank, uint32_t get_address, uint32_t *get_value)
{
	struct target *target = bank->target;
	int retValue = target_read_u32(target, get_address, get_value);
	return retValue;
}

static int outw(struct flash_bank *bank, uint32_t set_address, uint32_t set_value)
{
	struct target *target = bank->target;
	int retValue = target_write_u32(target, set_address, set_value);
	return retValue;
}

static int spib_get_regtiming(struct flash_bank *bank, unsigned int *get_value)
{
	if (inw(bank, SPIB_REG_REGTIMING, get_value) != ERROR_OK) {
		LOG_ERROR("read spib timing reg(addr:%x) failed", SPIB_REG_REGTIMING);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int spib_set_regtiming(struct flash_bank *bank, unsigned int data)
{
	if (outw(bank, SPIB_REG_REGTIMING, data) != ERROR_OK) {
		LOG_ERROR("write spib timing reg:%x value:%x failed", SPIB_REG_REGTIMING, data);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int spib_get_ctrl(struct flash_bank *bank, unsigned int *get_value)
{
	if (inw(bank, SPIB_REG_CTRL, get_value) != ERROR_OK) {
		LOG_ERROR("read spib ctrl reg(addr:%x) failed", SPIB_REG_CTRL);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int spib_set_ctrl(struct flash_bank *bank, unsigned int data)
{
	if (outw(bank, SPIB_REG_CTRL, data) != ERROR_OK) {
		LOG_ERROR("write spib ctrl reg:%x value:%x failed", SPIB_REG_CTRL, data);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int spib_get_data(struct flash_bank *bank, unsigned int *get_value)
{
	if (inw(bank, SPIB_REG_DATA, get_value) != ERROR_OK) {
		LOG_ERROR("read spib data reg(addr:%x) failed", SPIB_REG_DATA);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int spib_set_data(struct flash_bank *bank, unsigned int data)
{
	if (outw(bank, SPIB_REG_DATA, data) != ERROR_OK) {
		LOG_ERROR("write spib data reg:%x value:%x failed", SPIB_REG_DATA, data);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int spib_set_dctrl(struct flash_bank *bank, unsigned int data)
{
	if (outw(bank, SPIB_REG_DCTRL, data) != ERROR_OK) {
		LOG_ERROR("write spib dctrl reg:%x value:%x failed", SPIB_REG_DCTRL, data);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int spib_set_cmd(struct flash_bank *bank, unsigned int data)
{
	if (outw(bank, SPIB_REG_CMD, data) != ERROR_OK) {
		LOG_ERROR("write spib cmd reg:%x value:%x failed", SPIB_REG_CMD, data);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int spib_get_fifost(struct flash_bank *bank, unsigned int *get_value)
{
	if (inw(bank, SPIB_REG_FIFOST, get_value) != ERROR_OK) {
		LOG_ERROR("read spib status reg(addr:%x) failed", SPIB_REG_FIFOST);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static unsigned int spib_get_busy(struct flash_bank *bank)
{
	unsigned int reg = 0;
	if (spib_get_fifost(bank, &reg) != ERROR_OK)
		return 1;
	return reg & SPIB_FIFOST_SPIBSY_MASK;
}

static unsigned int spib_wait_spi(struct flash_bank *bank)
{
	unsigned int i;
	unsigned int timeout = FLASH_RETRY_TIMES;

	for (i = 1; i < timeout; i++) {
		if (spib_get_busy(bank) == 0)
			return 0;
		alive_sleep(1);
	}
	LOG_ERROR("spib_wait_spi: timeout\n");
	return 1;
}

static unsigned int spib_get_rx_empty(struct flash_bank *bank)
{
	unsigned int reg = 0;
	if (inw(bank, SPIB_REG_FIFOST, &reg) != ERROR_OK) {
		LOG_ERROR("read spib status_reg:%x failed", SPIB_REG_FIFOST);
		return 1;
	}
	return reg & SPIB_FIFOST_RXFEM_MASK;
}

static void spib_clr_fifo(struct flash_bank *bank)
{
	spib200_ctrl |= (SPIB_CTRL_TXFRST_MASK | SPIB_CTRL_RXFRST_MASK);
	spib_set_ctrl(bank, spib200_ctrl);
	spib_get_ctrl(bank, &spib200_ctrl);
	LOG_DEBUG("SPIB Control REG value = 0x%08x\n", spib200_ctrl);
}

static unsigned int spib_prepare_dctrl(
		unsigned int cmden,
		unsigned int addren,
		unsigned int tm,
		unsigned int wcnt,
		unsigned int dycnt,
		unsigned int rcnt)
{
	unsigned int v[8];
	unsigned int i;
	unsigned int dctrl = 0x0;

	v[0] = ((cmden << SPIB_DCTRL_CMDEN_OFFSET) & SPIB_DCTRL_CMDEN_MASK);
	v[1] = ((addren << SPIB_DCTRL_ADDREN_OFFSET) & SPIB_DCTRL_ADDREN_MASK);
	v[2] = ((tm << SPIB_DCTRL_TRAMODE_OFFSET) & SPIB_DCTRL_TRAMODE_MASK);
	v[3] = ((wcnt << SPIB_DCTRL_WCNT_OFFSET) & SPIB_DCTRL_WCNT_MASK);
	v[4] = ((dycnt << SPIB_DCTRL_DYCNT_OFFSET) & SPIB_DCTRL_DYCNT_MASK);
	v[5] = ((rcnt << SPIB_DCTRL_RCNT_OFFSET) & SPIB_DCTRL_RCNT_MASK);

	for (i = 0; i < 6; i++)
		dctrl |= v[i];
	return dctrl;
}

static unsigned int spirom_prepare_cmd(unsigned int cmd, unsigned int addr)
{
	unsigned int b0 = (cmd & 0xff);
	unsigned int b1 = (((addr >> 16) & 0xff) << 8);
	unsigned int b2 = (((addr >> 8) & 0xff) << 16);
	unsigned int b3 = ((addr & 0xff) << 24);
	unsigned int word = (b0 | b1 | b2 | b3);
	return word;
}

static int spib_exe_cmd(struct flash_bank *bank, unsigned int op_addr, unsigned int spib_dctrl)
{
	/*-- push flash command into tx fifo --*/
	if (spib_set_data(bank, op_addr) != ERROR_OK)
		return ERROR_FAIL;
	/*-- set dctrl --*/
	if (spib_set_dctrl(bank, spib_dctrl) != ERROR_OK)
		return ERROR_FAIL;
	/*-- set dummy command to trigger transation start --*/
	if (spib_set_cmd(bank, 0x0) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int spib_tx_data(struct flash_bank *bank, unsigned int *pTxdata, int TxBytes)
{
	unsigned int i, j;
	unsigned int TxWords = (TxBytes / 4);
	unsigned int *p_src_buffer = (unsigned int *)pTxdata;
	unsigned int timeout = FLASH_RETRY_TIMES;
	unsigned int reg, spib_tx_full;

	for (i = 0; i < TxWords; i++) {
		for (j = 0; j < timeout; j++) {
			if (spib_get_fifost(bank, &reg) != ERROR_OK)
				return ERROR_FAIL;
			spib_tx_full = reg & SPIB_FIFOST_TXFFL_MASK;
			if (spib_tx_full == 0)
				break;
		}
		if (spib_tx_full == 1) {
			LOG_ERROR("spib_set_fifo: write fifo timeout\n");
			return ERROR_FAIL;
		}
		if (spib_set_data(bank, *p_src_buffer++) != ERROR_OK)
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

static unsigned int spirom_cmd_send(
	struct flash_bank *bank,
	unsigned int cmd,
	unsigned int addr,
	unsigned int bytes,
	unsigned int *pdata,
	unsigned int *Retdata)
{
	unsigned int i;
	unsigned int timeout = FLASH_RETRY_TIMES;
	unsigned int spib_dctrl;
	unsigned int op_addr;
	unsigned int data = 0;
	unsigned int spib_busy = 0;
	unsigned int spib_rx_empty;

	/*-- wait if there is active transaction --*/
	spib_busy = spib_wait_spi(bank);
	if (spib_busy != 0) {
		*Retdata = 0;
		return 1;
	}
	/*-- clear tx/rx fifo --*/
	spib_clr_fifo(bank);

	if (cmd == SPIROM_CMD_RDID) {
		op_addr = SPIROM_OP_RDID;
		spib_dctrl = spib_prepare_dctrl(0x0, 0x0, SPIB_TM_WR_RD, 0, 0, 2);
		if (spib_exe_cmd(bank, op_addr, spib_dctrl) != ERROR_OK)
			return 1;
		/*-- wait data completion --*/
		for (i = 1; i < timeout; i++) {
			spib_rx_empty = spib_get_rx_empty(bank);
			if (spib_rx_empty == 0)
				break;
		}
		if (spib_rx_empty != 0) {
			LOG_ERROR("RDID timeout\n");
			*Retdata = 0;
			return 1;
		}
		if (spib_get_data(bank, &data) != ERROR_OK)
			return 1;
	} else if (cmd == SPIROM_CMD_RDST) {
		/*-- execute command --*/
		op_addr = SPIROM_OP_RDSR;
		spib_dctrl = spib_prepare_dctrl(0x0, 0x0, SPIB_TM_WR_RD, 0, 0, 0);
		if (spib_exe_cmd(bank, op_addr, spib_dctrl) != ERROR_OK)
			return 1;
		/*-- wait data completion --*/
		for (i = 1; i < timeout; i++) {
			spib_rx_empty = spib_get_rx_empty(bank);
			if (spib_rx_empty == 0)
				break;
		}
		if (spib_rx_empty != 0) {
			LOG_DEBUG("RDST timeout\n");
			*Retdata = 0;
			return 1;
		}
		if (spib_get_data(bank, &data) != ERROR_OK)
			return 1;
	} else if (cmd == SPIROM_CMD_WREN) {
		/*-- execute command --*/
		op_addr = SPIROM_OP_WREN;
		spib_dctrl = spib_prepare_dctrl(0x0, 0x0, SPIB_TM_WRonly, 0, 0, 0);
		if (spib_exe_cmd(bank, op_addr, spib_dctrl) != ERROR_OK)
			return 1;
		/*-- wait completion --*/
		spib_busy = spib_wait_spi(bank);
		if (spib_busy != 0) {
			LOG_ERROR("WREN timeout\n");
			*Retdata = 0;
			return 1;
		}
	} else if (cmd == SPIROM_CMD_ERASE) {
		/*-- execute command --*/
		op_addr = spirom_prepare_cmd(SPIROM_OP_SE, addr);
		spib_dctrl = spib_prepare_dctrl(0x0, 0x0, SPIB_TM_WRonly, 3, 0, 0);
		if (spib_exe_cmd(bank, op_addr, spib_dctrl) != ERROR_OK)
			return 1;
		/*-- wait completion --*/
		spib_busy = spib_wait_spi(bank);
		if (spib_busy != 0) {
			LOG_ERROR("ERASE timeout\n");
			*Retdata = 0;
			return 1;
		}
	} else if (cmd == SPIROM_CMD_PROGRAM) {
		/*-- execute command --*/
		op_addr = spirom_prepare_cmd(SPIROM_OP_PP, addr);
		spib_dctrl = spib_prepare_dctrl(0x0, 0x0, SPIB_TM_WRonly, 3+bytes, 0, 0);
		if (spib_exe_cmd(bank, op_addr, spib_dctrl) != ERROR_OK)
			return 1;
		/*-- write data --*/
		if (spib_tx_data(bank, pdata, bytes) != ERROR_OK)
			return 1;
	} else if (cmd == SPIROM_CMD_WRSR) {
		/*-- execute command --*/
		op_addr = (SPIROM_OP_WRSR | (addr<<8));
		spib_dctrl = spib_prepare_dctrl(0x0, 0x0, SPIB_TM_WRonly, 1, 0, 0);
		if (spib_exe_cmd(bank, op_addr, spib_dctrl) != ERROR_OK)
			return 1;
		/*-- wait completion --*/
		spib_busy = spib_wait_spi(bank);
		if (spib_busy != 0) {
			LOG_ERROR("ERASE timeout\n");
			*Retdata = 0;
			return 1;
		}
	} else {
		LOG_ERROR("wrong cmd\n");
		*Retdata = 0;
		return 1;
	}
	*Retdata = data;
	return 0;
}

static int platform_init(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);
	unsigned int smu_id_reg = 0, smu_base;

	spib200_base = CPE_SPIB_BASE;
	if (spib_get_ctrl(bank, &spib200_ctrl) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("SPIB Control REG value = 0x%08x\n", spib200_ctrl);

	smu_base = REG_SMU_BASE;
	if (inw(bank, smu_base, &smu_id_reg) != ERROR_OK) {
		LOG_ERROR("read smu_base:%x failed", smu_base);
		return ERROR_FAIL;
	}
	LOG_DEBUG("SMU_VER_ID = 0x%08x\n", smu_id_reg);
	return ERROR_OK;
}

static int mxic200_check(struct flash_bank *bank, unsigned int *get_id)
{
	LOG_DEBUG("%s", __func__);
	unsigned int spib_timing, result, RetData;
	/* Set SCLK period = ((0+1)*2)*(Period of the SPI clock source) */
	if (spib_get_regtiming(bank, &spib_timing) != ERROR_OK)
		return ERROR_FAIL;

	RetData = (spib_timing & (~0xFF));
	if (spib_set_regtiming(bank, RetData) != ERROR_OK)
		return ERROR_FAIL;

	if (spib_get_regtiming(bank, &spib_timing) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("mxic200_check: REGTIMING=%x\n", spib_timing);

	result = spirom_cmd_send(bank, SPIROM_CMD_RDID, 0x0, 0, NULL, &RetData);
	if (result != 0) {
		LOG_DEBUG("ERROR: read spi rom id fail\n");
		return ERROR_FAIL;
	}
	RetData &= SPIROM_ID_MASK;
	LOG_DEBUG("mxic200: ROM ID = 0x%08x\n", RetData);
	*get_id = RetData;
	return 0;
}

static int mxic200_erase(struct flash_bank *bank, unsigned int EraseSectorIndex)
{
	unsigned int EraseAddrStart, EraseSize, j;
	unsigned int result, RetData, timeout = FLASH_RETRY_TIMES;

	EraseAddrStart = bank->sectors[EraseSectorIndex].offset;
	EraseSize = bank->sectors[EraseSectorIndex].size;
	/*---------------------*/
	/*-- ERASE procedure   */
	/*---------------------*/
	for (j = 1; j < timeout; j++) {
		/*-- write enable --*/
		result = spirom_cmd_send(bank, SPIROM_CMD_WREN, 0x0, 0, NULL, &RetData);
		if (result != 0) {
			LOG_ERROR("mxic200_erase: (erase) enable write fail\n");
			return ERROR_FAIL;
		}
		/*-- get enable status --*/
		result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
		if (result != 0) {
			LOG_ERROR("mxic200_erase: (erase) get write enable status fail\n");
			return ERROR_FAIL;
		}
		if (RetData & SPIROM_SR_BP_MASK) {
			LOG_ERROR("mxic200_erase: Flash block locked.\n");
			return ERROR_FAIL;
		} else if (RetData & SPIROM_SR_WEL_MASK)
			break;
	}
	if ((RetData & SPIROM_SR_WEL_MASK) == 0) {
		LOG_ERROR("mxic200_erase: (erase) status %x, write enable is not set\n", RetData);
		return ERROR_FAIL;
	}

	LOG_DEBUG("erasing block %03d (0x%06x ~ 0x%06x)\n", EraseSectorIndex, EraseAddrStart, EraseAddrStart + EraseSize);

	/*-- erase --*/
	result = spirom_cmd_send(bank, SPIROM_CMD_ERASE, EraseAddrStart, 0, NULL, &RetData);
	if (result != 0) {
		LOG_ERROR("mxic200_erase: rom erase fail\n");
		return ERROR_FAIL;
	}

	/*-- get erase status --*/
	for (j = 1; j < timeout; j++) {
		result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
		if (result != 0) {
			LOG_ERROR("mxic200_erase: get erase status fail\n");
			return ERROR_FAIL;
		}
		if ((RetData & SPIROM_SR_WIP_MASK) == 0)
			break;
	}
	if ((RetData & SPIROM_SR_WIP_MASK) != 0) {
		LOG_ERROR("mxic200_erase: status %x, erase is still in progress\n", RetData);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int mxic200_set_wrsr(struct flash_bank *bank, unsigned int uiStat)
{
	LOG_DEBUG("%s", __func__);
	unsigned int i, j, RetData, timeout = FLASH_RETRY_TIMES, result;

	for (i = 1; i < timeout; i++) {
		/*-- write enable --*/
		result = spirom_cmd_send(bank, SPIROM_CMD_WREN, 0x0, 0, NULL, &RetData);
		if (result != 0) {
			LOG_ERROR("mxic200_set_wrsr: (lock/unlock) enable write fail\n");
			return ERROR_FAIL;
		}

		/*-- get enable status --*/
		for (j = 1; j < timeout; j++) {
			result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
			if (result != 0) {
				LOG_ERROR("mxic200_set_wrsr: (lock/unlock) get write enable status fail\n");
				return ERROR_FAIL;
			}
			if ((RetData & SPIROM_SR_WIP_MASK) == 0)
				break;
		}
		if (RetData & SPIROM_SR_WEL_MASK)
			break;
	}
	if ((RetData & SPIROM_SR_WEL_MASK) == 0) {
		LOG_ERROR("mxic200_set_wrsr: (lock/unlock) status %x, write enable is not set\n", RetData);
		return ERROR_FAIL;
	}
	/*-- set WRSR --*/
	result = spirom_cmd_send(bank, SPIROM_CMD_WRSR, uiStat, 0, NULL, &RetData);
	if (result != 0) {
		LOG_ERROR("mxic200_set_wrsr: (lock/unlock) write BP0~3 value fail\n");
		return ERROR_FAIL;
	}

	/*-- get status --*/
	for (j = 1; j < timeout; j++) {
		result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
		if (result != 0) {
			LOG_ERROR("mxic200_set_wrsr: (lock/unlock) get status fail\n");
			return ERROR_FAIL;
		}
		if ((RetData & SPIROM_SR_WIP_MASK) == 0)
			break;
	}
	return ERROR_OK;
}

static int mxic200_lock(struct flash_bank *bank, unsigned int FlashAddr, unsigned int DataSize)
{
	LOG_DEBUG("%s", __func__);
	unsigned int RetData = 0, result;
	result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
	if (result != 0) {
		LOG_ERROR("mxic200_lock: read status fail\n");
		return ERROR_FAIL;
	}
	if (mxic200_set_wrsr(bank, RetData | 0x3c) != ERROR_OK) {
		LOG_ERROR("mxic200_lock: set lock fail\n");
		return ERROR_FAIL;
	}
	result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
	if (result != 0) {
		LOG_ERROR("mxic200_lock: read status fail\n");
		return ERROR_FAIL;
	}
	if ((RetData & 0x3c) != 0x3c) {
		LOG_ERROR("mxic200_lock check lock status = %08x\n", RetData);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int mxic200_unlock(struct flash_bank *bank, unsigned int FlashAddr, unsigned int DataSize)
{
	LOG_DEBUG("%s", __func__);
	unsigned int RetData = 0, result;
	result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
	if (result != 0) {
		LOG_ERROR("mxic200_unlock: read status fail\n");
		return ERROR_FAIL;
	}
	if (mxic200_set_wrsr(bank, RetData & ~0x3c) != ERROR_OK) {
		LOG_ERROR("mxic200_unlock: set unlock fail\n");
		return ERROR_FAIL;
	}
	result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
	if (result != 0) {
		LOG_ERROR("mxic200_unlock: read status fail\n");
		return ERROR_FAIL;
	}
	if ((RetData & 0x3c) != 0) {
		LOG_ERROR("mxic200_unlock check unlock status = %08x\n", RetData);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int mxic200_page_program(
		struct flash_bank *bank,
		unsigned int FlashAddr,
		unsigned char *start,
		unsigned int PageCnt)
{
	unsigned int result, RetData, i;
	unsigned int j, timeout = FLASH_RETRY_TIMES, page_size = 0;
	struct ndsspi200_flash_bank *ndsspi200_info = bank->driver_priv;

	page_size = ndsspi200_info->dev->pagesize;
	/*---------------------------*/
	/*-- PAGE PROGRAM procedure  */
	/*---------------------------*/
	for (i = 0; i < PageCnt; i++) {
		printf("program addr = 0x%x, cur_page = 0x%x ...\n", FlashAddr, i);
		fflush(stdout);
		/*-- write enable --*/
		for (j = 1; j < timeout; j++) {
			result = spirom_cmd_send(bank, SPIROM_CMD_WREN, 0x0, 0, NULL, &RetData);
			if (result != 0) {
				LOG_ERROR("(program) page %d enable write fail\n", i);
				return ERROR_FAIL;
			}
			/*-- get enable status --*/
			result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
			if (result != 0) {
				LOG_ERROR("get (program) page %d write enable status fail\n", i);
				return ERROR_FAIL;
			}
			if (RetData & SPIROM_SR_BP_MASK) {
				LOG_ERROR("Flash block locked.\n");
				return ERROR_FAIL;
			} else if (RetData & SPIROM_SR_WEL_MASK)
				break;
		}
		if ((RetData & SPIROM_SR_WEL_MASK) == 0) {
			LOG_ERROR("(program) page %d, write enable is not set (status %x)\n", i, RetData);
			return ERROR_FAIL;
		}
		result = spirom_cmd_send(bank, SPIROM_CMD_PROGRAM, FlashAddr, page_size, (unsigned int *)start, &RetData);
		if (result != 0) {
			LOG_ERROR("(program) page %d fail\n", i);
			return ERROR_FAIL;
		}
		/*-- ckeck completion --*/
		for (j = 1; j < timeout; j++) {
			result = spirom_cmd_send(bank, SPIROM_CMD_RDST, 0x0, 0, NULL, &RetData);
			if (result != 0) {
				LOG_ERROR("get (program) page %d write enable status fail\n", i);
				return ERROR_FAIL;
			}
			if ((RetData & SPIROM_SR_WIP_MASK) == 0)
				break;
		}
		if ((RetData & SPIROM_SR_WEL_MASK) != 0) {
			LOG_ERROR("(program) page %d, write enable is not clear (status %x)\n", i, RetData);
			return ERROR_FAIL;
		}
		start += page_size;
		FlashAddr += page_size;
	}  /* for (i = 0; i < PageCnt; i++) */
	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(ndsspi200_flash_bank_command)
{
	LOG_DEBUG("%s", __func__);
	LOG_DEBUG("%s, %s, base=0x%" TARGET_PRIxADDR ", size=0x%08x",
			bank->name, bank->driver->name, bank->base, bank->size);

	struct ndsspi200_flash_bank *ndsspi200_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ndsspi200_info = malloc(sizeof(struct ndsspi200_flash_bank));
	if (ndsspi200_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = ndsspi200_info;
	ndsspi200_info->probed = 0;

	return ERROR_OK;
}

static int ndsspi200_probe(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);

	struct ndsspi200_flash_bank *ndsspi200_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id = 0;
	int retval;

	if (ndsspi200_info->probed)
		free(bank->sectors);
	ndsspi200_info->probed = 0;

	if (platform_init(bank) != ERROR_OK)
		return ERROR_FAIL;

	/* read and decode flash ID; returns in SW mode */
	retval = mxic200_check(bank, &id);
	if (retval != ERROR_OK)
		return retval;

	ndsspi200_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			ndsspi200_info->dev = (struct flash_device *)p;
			break;
		}

	if (!ndsspi200_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		printf("Unknown flash device (ID 0x%08" PRIx32 ")\n", id);
		fflush(stdout);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08x), baseaddr = 0x%" TARGET_PRIxADDR,
			ndsspi200_info->dev->name, ndsspi200_info->dev->device_id, bank->base);
	LOG_INFO("size_in_bytes 0x%x", (unsigned int)ndsspi200_info->dev->size_in_bytes);
	LOG_INFO("sectorsize 0x%x", (unsigned int)ndsspi200_info->dev->sectorsize);
	LOG_INFO("pagesize 0x%x", ndsspi200_info->dev->pagesize);

	/* Set correct size value */
	bank->size = ndsspi200_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors =
		ndsspi200_info->dev->size_in_bytes / ndsspi200_info->dev->sectorsize;
	LOG_INFO("number of sectors 0x%x", bank->num_sectors);
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}
	/* SW protect */
	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * ndsspi200_info->dev->sectorsize;
		sectors[sector].size = ndsspi200_info->dev->sectorsize;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	ndsspi200_info->probed = 1;
	return ERROR_OK;
}

static int ndsspi200_auto_probe(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);

	struct ndsspi200_flash_bank *ndsspi200_info = bank->driver_priv;

	if (ndsspi200_info->probed)
		return ERROR_OK;

	return ndsspi200_probe(bank);
}

static int ndsspi200_protect(struct flash_bank *bank, int set,
		int first, int last)
{
	LOG_DEBUG("%s", __func__);
	int sector, result;

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;

	/* SMP mode */
	if (set)
		result = mxic200_lock(bank, 0, 0);
	else
		result = mxic200_unlock(bank, 0, 0);

	return result;
}

static struct algorithm_steps *ndsspi_as_new(uint32_t size)
{
	struct algorithm_steps *as = calloc(1, sizeof(struct algorithm_steps));
	as->size = size;
	as->steps = calloc(size, sizeof(as->steps[0]));
	return as;
}

static struct algorithm_steps *ndsspi_as_delete(struct algorithm_steps *as)
{
	for (unsigned step = 0; step < as->used; step++) {
		free(as->steps[step]);
		as->steps[step] = NULL;
	}
	free(as->steps);
	free(as);
	return NULL;
}

static int ndsspi_as_empty(struct algorithm_steps *as)
{
	for (uint32_t s = 0; s < as->used; s++) {
		if (as->steps[s][0] != STEP_NOP)
			return 0;
	}
	return 1;
}

/* Return size of compiled program. */
static uint32_t ndsspi_as_compile(struct algorithm_steps *as, uint8_t *target,
		uint32_t target_size, uint32_t page_size)
{
	uint32_t offset = 0;
	bool finish_early = false;
	uint32_t tx_size = page_size;

	LOG_DEBUG("as->used %d", as->used);
	for (uint32_t s = 0; s < as->used && !finish_early; s++) {
		uint32_t bytes_left = target_size - offset;
		LOG_DEBUG("as->steps[%d][0] 0x%x", s, as->steps[s][0]);

		switch (as->steps[s][0]) {
			case STEP_NOP:
				break;
			case STEP_TX:
				if ((tx_size + tx_used + 1) > bytes_left) {
					if (s == 0)
						LOG_ERROR("allocate size for data too small");
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], tx_size + tx_used);
				offset += tx_size + tx_used;
				break;
			case STEP_ERASE:
				if ((erase_used + 1) > bytes_left) {
					if (s == 0)
						LOG_ERROR("allocate size for data too small");
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], erase_used);
				offset += erase_used;
				break;
			default:
				assert(0);
		}
		if (!finish_early)
			as->steps[s][0] = STEP_NOP;
	}
	assert(offset + 1 <= target_size);
	target[offset++] = STEP_EXIT;

	LOG_DEBUG("%d-byte program:", offset);
	for (uint32_t i = 0; i < offset;) {
		char buf[80];
		for (uint32_t x = 0; i < offset && x < 16; x++, i++)
			sprintf(buf + x*3, "%02x ", target[i]);
		LOG_DEBUG("%s", buf);
	}

	return offset;
}

static void ndsspi_as_add_erase(struct algorithm_steps *as, uint32_t addr)
{
	LOG_DEBUG("sector_addr=%d, as->used %d", addr, as->used);
	uint32_t op_addr = spirom_prepare_cmd(SPIROM_OP_SE, addr);
	assert(as->used < as->size);
	as->steps[as->used] = malloc(erase_used);
	as->steps[as->used][0] = STEP_ERASE;
	as->steps[as->used][1] = (op_addr & 0xff);
	as->steps[as->used][2] = (op_addr & 0xff00) >> 8;
	as->steps[as->used][3] = (op_addr & 0xff0000) >> 16;
	as->steps[as->used][4] = (op_addr & 0xff000000) >> 24;
	as->used++;
}

static void ndsspi_as_add_tx(
		struct algorithm_steps *as,
		uint32_t offset,
		uint32_t tx_count,
		const uint8_t *tx_data,
		uint32_t page_size)
{
	LOG_DEBUG("tx_count=%d, as->used %d", tx_count, as->used);
	uint32_t step_count = page_size, op_addr;
	while (tx_count > 0) {
		op_addr = spirom_prepare_cmd(SPIROM_OP_PP, offset);
		assert(as->used < as->size);
		as->steps[as->used] = malloc(step_count + tx_used);
		as->steps[as->used][0] = STEP_TX;
		as->steps[as->used][1] = (op_addr & 0xff);
		as->steps[as->used][2] = (op_addr & 0xff00) >> 8;
		as->steps[as->used][3] = (op_addr & 0xff0000) >> 16;
		as->steps[as->used][4] = (op_addr & 0xff000000) >> 24;
		memcpy(as->steps[as->used] + tx_used, tx_data, step_count);
		as->used++;
		tx_data += step_count;
		offset += step_count;
		tx_count -= step_count;
	}
}

static void exec_fencei(struct target *target)
{
	/* send fence.i check not write to cache */
	LOG_DEBUG("send fence.i");
	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_fence_i(&program);
	if (riscv_program_exec(&program, target) != ERROR_OK)
		LOG_ERROR("Unable to execute fence.i");
}

static int ndsspi_steps_execute(
		struct algorithm_steps *as,
		struct flash_bank *bank,
		struct working_area *algorithm_wa,
		struct working_area *data_wa)
{
	LOG_DEBUG("%s", __func__);

	struct target *target = bank->target;
	struct ndsspi200_flash_bank *ndsspi200_info = bank->driver_priv;
	uint32_t page_size = ndsspi200_info->dev->pagesize;
	struct reg_param reg_params[1];
	int num_used_reg = 0, retval = ERROR_OK;

	int xlen = riscv_xlen(target);

	/* use a0 to store cmd address */
	num_used_reg = 1;
	init_reg_param(&reg_params[0], "a0", xlen, PARAM_OUT);
	buf_set_u64(reg_params[0].value, 0, xlen, data_wa->address);

	while (!ndsspi_as_empty(as)) {
		keep_alive();
		uint8_t *data_buf = malloc(data_wa->size);
		uint32_t bytes = ndsspi_as_compile(as, data_buf, data_wa->size, page_size);
		LOG_DEBUG("write data to 0x%" TARGET_PRIxADDR ": %d bytes", data_wa->address, bytes);
		if (bytes == 1) {
			LOG_ERROR("only get 1 byte data");
			goto err;
		}

		exec_fencei(target);
		retval = target_write_buffer(target, data_wa->address, bytes, data_buf);
		free(data_buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write data to 0x%" TARGET_PRIxADDR ": %d", data_wa->address, retval);
			goto err;
		}
		retval = target_run_algorithm(target, 0, NULL, num_used_reg, reg_params,
				algorithm_wa->address, algorithm_wa->address + STEP_EXIT , 100000, NULL);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d", algorithm_wa->address, retval);
			goto err;
		}
	}

	LOG_DEBUG("target_run_algorithm finish !!");
err:
	return retval;
}

static int ndsspi200_erase(struct flash_bank *bank, int first, int last)
{
	LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);

	struct target *target = bank->target;
	struct ndsspi200_flash_bank *ndsspi200_info = bank->driver_priv;
	int retval = ERROR_OK;
	int sector;
	unsigned int EraseAddrStart;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(ndsspi200_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* SW protect */
	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	struct working_area *nds_algorithm_wa;
	if (target_alloc_working_area(target, sizeof(algorithm_bin),
					&nds_algorithm_wa) != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area.",
					sizeof(algorithm_bin));
		nds_algorithm_wa = NULL;
	} else {
		exec_fencei(target);
		retval = target_write_buffer(target, nds_algorithm_wa->address, sizeof(algorithm_bin), algorithm_bin);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write code to 0x%" TARGET_PRIxADDR ": %d", nds_algorithm_wa->address, retval);
			target_free_working_area(target, nds_algorithm_wa);
			nds_algorithm_wa = NULL;
		} else {
			LOG_DEBUG("write code to 0x%" TARGET_PRIxADDR ": 0x%x bytes", nds_algorithm_wa->address,
					(unsigned int)sizeof(algorithm_bin));
		}
	}

	struct working_area *nds_data_wa = NULL;
	uint32_t data_wa_size = (last - first + 1) * erase_used + 1;
	while (1) {
		if (data_wa_size < (erase_used + 1)) {
			LOG_ERROR("Couldn't allocate the small data working area: %d.", erase_used + 1);
			target_free_working_area(target, nds_algorithm_wa);
			nds_algorithm_wa = NULL;
		}
		if (target_alloc_working_area_try(target, data_wa_size, &nds_data_wa) == ERROR_OK)
			break;
		data_wa_size /= 2;
	}

	/* algorithm mode */
	if (nds_algorithm_wa) {
		struct algorithm_steps *nds_as = ndsspi_as_new(last - first + 1);
		for (sector = first; sector <= last; sector++) {
			EraseAddrStart = bank->sectors[sector].offset;
			ndsspi_as_add_erase(nds_as, EraseAddrStart);
		}
		retval = ndsspi_steps_execute(nds_as, bank, nds_algorithm_wa, nds_data_wa);
		if (retval != ERROR_OK)
			LOG_DEBUG("algorithm erase failed");

		target_free_working_area(target, nds_algorithm_wa);
		target_free_working_area(target, nds_data_wa);
		ndsspi_as_delete(nds_as);
	} else {
		for (sector = first; sector <= last; sector++) {
			retval = mxic200_erase(bank, sector);
			if (retval != ERROR_OK) {
				LOG_DEBUG("slow mode erase failed");
				break;
			}
			keep_alive();
		}
	}

	return retval;
}

static int ndsspi200_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	LOG_DEBUG("offset=0x%08x, count=0x%08x", offset, count);
	return target_read_buffer(bank->target, offset + bank->base, count, buffer);
}

static int ndsspi200_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct ndsspi200_flash_bank *ndsspi200_info = bank->driver_priv;
	uint32_t cur_count, cur_offset, page_size, page_offset, cmd_size;
	int sector;
	int retval = ERROR_OK;
	uint8_t *tmp_buffer = NULL;

	LOG_DEBUG("offset=0x%08x, count=0x%08x", offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		printf("Target not halted\n");
		fflush(stdout);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > ndsspi200_info->dev->size_in_bytes) {
		LOG_WARNING("Write past end of flash. Extra data discarded.");
		count = ndsspi200_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
					(bank->sectors[sector].offset + bank->sectors[sector].size))
				&& ((offset + count - 1) >= bank->sectors[sector].offset)
				&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			printf("Flash sector %d protected\n", sector);
			fflush(stdout);
			return ERROR_FAIL;
		}
	}

	page_size = ndsspi200_info->dev->pagesize;
	tmp_buffer = malloc(page_size);
	if (tmp_buffer == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	struct working_area *nds_algorithm_wa;
	if (target_alloc_working_area(target, sizeof(algorithm_bin),
					&nds_algorithm_wa) != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area.",
					sizeof(algorithm_bin));
		nds_algorithm_wa = NULL;
	} else {
		exec_fencei(target);
		retval = target_write_buffer(target, nds_algorithm_wa->address, sizeof(algorithm_bin), algorithm_bin);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write code to 0x%" TARGET_PRIxADDR ": %d", nds_algorithm_wa->address, retval);
			target_free_working_area(target, nds_algorithm_wa);
			nds_algorithm_wa = NULL;
		} else {
			LOG_DEBUG("write code to 0x%" TARGET_PRIxADDR ": 0x%x bytes",
					nds_algorithm_wa->address, (unsigned int)sizeof(algorithm_bin));
		}
	}

	struct working_area *nds_data_wa = NULL;
	if (count < page_size)
		cmd_size = page_size * 2;
	else
		cmd_size = count * 2;
	uint32_t data_wa_size = cmd_size;
	while (1) {
		if (data_wa_size < (page_size + tx_used + 1)) {
			LOG_ERROR("Couldn't allocate the small data working area: %d.", page_size + tx_used + 1);
			target_free_working_area(target, nds_algorithm_wa);
			nds_algorithm_wa = NULL;
		}
		if (target_alloc_working_area_try(target, data_wa_size, &nds_data_wa) == ERROR_OK)
			break;
		data_wa_size /= 2;
	}

	/* add 5 for offset not page_aligned or count%page_size not 0 */
	struct algorithm_steps *nds_as = ndsspi_as_new(count / page_size + 5);

	/* unaligned(page) buffer head */
	if ((offset % page_size) != 0) {
		/* read the 1st-unaligned-page, memcpy unaligned data */
		page_offset = (offset/page_size) * page_size;
		ndsspi200_read(bank, tmp_buffer, page_offset, page_size);

		cur_offset = (offset % page_size);
		cur_count = (page_size - cur_offset);
		if (cur_count > count)
			cur_count = count;

		memcpy(tmp_buffer+cur_offset, buffer, cur_count);

		if (nds_algorithm_wa)
			ndsspi_as_add_tx(nds_as, page_offset, page_size, tmp_buffer, page_size);
		else {
			retval = mxic200_page_program(bank, page_offset, (unsigned char *)tmp_buffer, 1);
			if (retval != ERROR_OK)
				goto ndsspi200_write_finish;
		}

		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
	}

	/* central part, aligned words */
	while (count >= page_size) {
		cur_count = (count/page_size) * page_size;

		if (nds_algorithm_wa)
			ndsspi_as_add_tx(nds_as, offset, cur_count, buffer, page_size);
		else {
			retval = mxic200_page_program(bank, offset, (unsigned char *)buffer,
					(cur_count + (page_size - 1)) / page_size);
			if (retval != ERROR_OK)
				goto ndsspi200_write_finish;
		}

		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
	}

	/* buffer tail */
	if (count > 0) {
		/* read the last-unaligned-page, memcpy unaligned data */
		ndsspi200_read(bank, tmp_buffer, offset, page_size);
		memcpy(tmp_buffer, buffer, count);
		if (nds_algorithm_wa)
			ndsspi_as_add_tx(nds_as, offset, page_size, tmp_buffer, page_size);
		else {
			retval = mxic200_page_program(bank, offset, (unsigned char *)tmp_buffer, 1);
			if (retval != ERROR_OK)
				goto ndsspi200_write_finish;
		}
	}

	if (nds_algorithm_wa)
		retval = ndsspi_steps_execute(nds_as, bank, nds_algorithm_wa, nds_data_wa);

ndsspi200_write_finish:
	free(tmp_buffer);
	ndsspi_as_delete(nds_as);
	if (nds_algorithm_wa) {
		target_free_working_area(target, nds_algorithm_wa);
		target_free_working_area(target, nds_data_wa);
	}
	return retval;
}

static int ndsspi200_protect_check(struct flash_bank *bank)
{
	LOG_DEBUG("%s", __func__);
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int ndsspi200_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	LOG_DEBUG("%s", __func__);
	struct ndsspi200_flash_bank *ndsspi200_info = bank->driver_priv;

	if (!(ndsspi200_info->probed)) {
		snprintf(buf, buf_size,
				"\nNDSSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nNDSSPI flash information:\n"
			"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
			ndsspi200_info->dev->name, ndsspi200_info->dev->device_id);

	return ERROR_OK;
}

const struct flash_driver ndsspi200_flash = {
	.name = "ndsspi200",
	.flash_bank_command = ndsspi200_flash_bank_command,
	.auto_probe = ndsspi200_auto_probe,
	.probe = ndsspi200_probe,
	.protect = ndsspi200_protect,
	.erase = ndsspi200_erase,
	.read = ndsspi200_read,
	.write = ndsspi200_write,
	.erase_check = default_flash_blank_check,
	.protect_check = ndsspi200_protect_check,
	.info = ndsspi200_get_info,
};
