/***************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>,	   *
 *   2017 - 2019 by Andreas Bolsch <andreas.bolsch@mni.thm.de			   *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
 *   (at your option) any later version.								   *
 *																		   *
 *   This program is distributed in the hope that it will be useful,	   *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *   GNU General Public License for more details.						   *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License	   *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/image.h>
#include "cmspi.h"
#include "sfdp.h"

#define I2C_ADDR_EEPROM_NORM 0xA0

/* for accessing byte offsets, extract offset byte, sign extent and shift left by 2 positions */
#define INPUT_OFF (((int32_t) ((int8_t) (cmspi_info->inp_off & 0xFF))) << 2)
#define DIR_INP_OFF(index) (((int32_t) ((int8_t) ((cmspi_info->dir_inp_off >> (index << 3)) & 0xFF))) << 2)
#define DIR_OUT_OFF(index) (((int32_t) ((int8_t) ((cmspi_info->dir_out_off >> (index << 3)) & 0xFF))) << 2)

#define CLR_PORT_BIT(data, pin)												\
{																			\
	(data) &= ~(cmspi_info->pin.mask);										\
	retval = target_write_u32(target, cmspi_info->pin.addr, data);			\
}

#define SET_PORT_BIT(data, pin)												\
{																			\
	(data) |= (cmspi_info->pin.mask);										\
	retval = target_write_u32(target, cmspi_info->pin.addr, data);			\
}

#define _CLR_I2C(data, pdir, pin, index)									\
{																			\
	if (cmspi_info->dir_inp_off == cmspi_info->dir_out_off) {				\
		/* read-modify-write direction register */							\
		if ((data & cmspi_info->pin.mask) != 0)								\
			pdir ^= cmspi_info->io[index].mask;								\
		(data) &= ~(cmspi_info->pin.mask);									\
		retval = target_write_u32(target, cmspi_info->pin.addr, data);		\
		if (retval == ERROR_OK)												\
			retval = target_write_u32(target, cmspi_info->io[index].addr	\
				+ DIR_INP_OFF(index), pdir);								\
	} else {																\
		/* set/clear direction register */									\
		(data) &= ~(cmspi_info->pin.mask);									\
		retval = target_write_u32(target, cmspi_info->pin.addr, data);		\
		if (retval == ERROR_OK)												\
			retval = target_write_u32(target, cmspi_info->io[index].addr	\
				+ DIR_OUT_OFF(index), cmspi_info->io[index].mask);			\
	}																		\
}

#define _SET_I2C(data, pdir, pin, index)									\
{																			\
	if (cmspi_info->dir_inp_off == cmspi_info->dir_out_off) {				\
		/* read-modify-write direction register */							\
		if ((data & cmspi_info->pin.mask) == 0)								\
			pdir = (pdir & ~cmspi_info->io[index].mask) |					\
				(cmspi_info->io[index].mask & cmspi_info->io_def[index]);	\
		retval = target_write_u32(target, cmspi_info->io[index].addr		\
			+ DIR_INP_OFF(index), pdir);									\
		(data) |= (cmspi_info->pin.mask);									\
		if (retval == ERROR_OK)												\
			retval = target_write_u32(target, cmspi_info->pin.addr, data);	\
	} else {																\
		/* set/clear direction register */									\
		retval = target_write_u32(target, cmspi_info->io[index].addr		\
			+ DIR_INP_OFF(index), cmspi_info->io[index].mask);				\
		(data) |= (cmspi_info->pin.mask);									\
		if (retval == ERROR_OK)												\
			retval = target_write_u32(target, cmspi_info->pin.addr, data);	\
	}																		\
}

#define CLR_SDA(data, pdir)		_CLR_I2C(data, pdir, SPI_SDA, I2C_SDA)
#define SET_SDA(data, pdir)		_SET_I2C(data, pdir, SPI_SDA, I2C_SDA)
#define CLR_SCL(data, pdir)		_CLR_I2C(data, pdir, SPI_SCL, I2C_SCL)
#define SET_SCL(data, pdir)		_SET_I2C(data, pdir, SPI_SCL, I2C_SCL)

/* number of bytes required for address */
#define SPI_ADDR_BYTES														\
	((log2u(cmspi_info->dev.size_in_bytes) + 7) / 8)

/* number of bytes required for address */
#define I2C_ADDR_BYTES														\
	((log2u(cmspi_info->dev.size_in_bytes) - cmspi_info->i2c_bits + 7) >> 3)

#define PARAM(field) h_to_le_32(cmspi_info->field)

/* SPI mode: lowest byte only, DPI: lowest halfword, QPI: whole word */
#define OFFS_USAGE(field) h_to_le_32(cmspi_info->field &					\
	((cmspi_info->mode == MODE_SPI) ? 0x000000FF :							\
	((cmspi_info->mode == MODE_DPI) ? 0x0000FFFF : 0xFFFFFFFF)))			\

#define FILL_PORT_BUFFER(spi_cmd)											\
	{																		\
		h_to_le_32((cmspi_info->inp_off << 16) |							\
			(cmspi_info->dummy << 8) | (spi_cmd)), PARAM(bit_no),			\
		OFFS_USAGE(dir_inp_off), OFFS_USAGE(dir_out_off),					\
		PARAM(ncs.addr), PARAM(ncs.mask),									\
		PARAM(clk.addr), PARAM(clk.mask),									\
		PARAM(io[0].addr), PARAM(io[0].mask),								\
		PARAM(io[1].addr), PARAM(io[1].mask),								\
		PARAM(io[2].addr), PARAM(io[2].mask),								\
		PARAM(io[3].addr), PARAM(io[3].mask),								\
	}

/* find first (and only!) bit set, gives the number of address bits */
static int log2u(uint32_t word)
{
	unsigned int result;

	for (result = 0; result < (sizeof(uint32_t) * CHAR_BIT); result++)
		if (word == (1UL << result))
			return result;

	return -1;
}

/* convert uint32_t into 4 uint8_t in little endian byte order */
static inline uint32_t h_to_le_32(uint32_t val)
{
	uint32_t result;

	h_u32_to_le((uint8_t *) &result, val);
	return result;
}

/* timeout in ms */
#define CMSPI_CMD_TIMEOUT			(100)
#define CMSPI_PROBE_TIMEOUT			(100)
#define CMSPI_MAX_TIMEOUT			(2000)
#define CMSPI_MASS_ERASE_TIMEOUT	(400000)

#define DATA_PINS					4

typedef struct {
	uint32_t addr;				/* address port *output* register */
	uint32_t mask;				/* bit mask for pin direction exor mask */
} port_pin;

typedef enum {
	MODE_SPI = 0,				/* IO1/MISO and IO0/MOSI or combined bidirectional IO0/SIO */
	MODE_DPI,					/* bidirectional IO1/MISO, IO0/MOSI */
	MODE_QPI,					/* bidirectional IO3/NHOLD, IO2/NWP, IO1/MISO, IO0/MOSI */
	MODE_I2C,					/* IO1/SDA, IO0/SCL */
} serial_mode_t;

struct cmspi_flash_bank {
	bool probed;
	uint32_t bank_num;
	char devname[32];
	struct flash_device dev;
	int sfdp_dummy;				/* number of dummy *bytes* for SFDP read */
	serial_mode_t mode;			/* current operating mode */
	uint8_t dummy;				/* number of dummy clocks, only in QPI read */
	uint8_t inp_off;			/* word offset port output reg. to port input reg. */
	uint8_t i2c_addr;			/* I2C address byte, left justified, bit 0 (r/w) must be zero  */
	uint8_t i2c_bits;			/* memory address bits in address byte */
	port_pin ncs;
	port_pin clk;
	port_pin io[DATA_PINS];		/* IO0/MOSI, IO1/MISO, IO2/NWP, IO3/NHOLD */
	uint32_t bit_no;			/* bit numbers of (msb) io3, io2, io1, io0 (lsb) */
	uint32_t dir_inp_off;		/* word offsets output reg. to port dir *CLR* reg. for io3, ..., io0 */
	uint32_t dir_out_off;		/* word offsets output reg. to port dir *SET* reg. for io3, ..., io0 */
	uint32_t io_def[DATA_PINS];	/* default directions settings */
	bool same_port;				/* whether SDA and SCL have same input/output registers */
	bool same_conf;				/* whether SDA and SCL have same config registers */
};

struct sector_info {
	uint32_t offset;
	uint32_t size;
	uint32_t result;
};

/* in SPI and DPI mode IO3/NHOLD and IO2/NWP must be kept high */
static int deactivate_nhold_nwp(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port_iox, dir_inp_off, dir_out_off;
	int k, retval = ERROR_OK;

	LOG_DEBUG("deactivating NHOLD and NWP");

	for (k = 3; k >= 2 ; --k) {
		if (cmspi_info->io[k].addr != 0) {
			/* set port value to high  */
			retval = target_read_u32(target, cmspi_info->io[k].addr, &port_iox);
			if (retval != ERROR_OK) {
				LOG_ERROR("reading io[%d] port data failed", k);
				return retval;
			}
			if (!(port_iox & cmspi_info->io[k].mask)) {
				port_iox |= cmspi_info->io[k].mask;
				retval = target_write_u32(target, cmspi_info->io[k].addr, port_iox);
				if (retval != ERROR_OK) {
					LOG_ERROR("writing io[%d] port data failed", k);
					return retval;
				}
			}

			dir_inp_off = DIR_INP_OFF(k);
			if (dir_inp_off) {
				dir_out_off = DIR_OUT_OFF(k);
				if (dir_inp_off == dir_out_off) {
					/* if dir_inp_off equals dir_out_off, use read-modify-write
					 * for direction register */
					retval = target_write_u32(target, cmspi_info->io[k].addr + dir_inp_off,
						cmspi_info->io_def[k] ^ cmspi_info->io[k].mask);
				} else {
					/* else use set/clear registers */
					retval = target_write_u32(target, cmspi_info->io[k].addr + dir_out_off,
						cmspi_info->io[k].mask);
				}
				if (retval != ERROR_OK) {
					LOG_ERROR("setting io[%d] direction failed", k);
					return retval;
				}
			}
		}
	}

	return retval;
}

/* set data pin direction to output or default (i. e. input) mode */
/* in QPI-mode all data pins are affected, in DPI-mode only IO1/MISO
 * and IO0/MOSI, in SPI mode only IO0/MOSI */
static int set_to_output(struct flash_bank *bank, bool out)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t dir_inp_off, dir_out_off;
	int k, retval = ERROR_OK;

	switch (cmspi_info->mode) {
		case MODE_SPI:
			LOG_DEBUG("spi mode, direction: %s", out ? "out" : "in");
			k = 1;	/* one data out pin */
			break;

		case MODE_DPI:
			LOG_DEBUG("dpi mode, direction: %s", out ? "out" : "in");
			k = 2;	/* two data out pins */
			break;

		case MODE_QPI:
			LOG_DEBUG("qpi mode, direction: %s", out ? "out" : "in");
			k = 4;	/* four data out pins */
			break;

		default:
			return ERROR_FAIL;
	}

	k--;
	do {
		dir_inp_off = DIR_INP_OFF(k);
		if (dir_inp_off) {
			dir_out_off = DIR_OUT_OFF(k);
			if (dir_inp_off == dir_out_off) {
				/* if dir_inp_off equals dir_out_off, use read-modify-write
				 * for direction register */
				retval = target_write_u32(target, cmspi_info->io[k].addr + dir_inp_off,
					out ? (cmspi_info->io_def[k] ^ cmspi_info->io[k].mask) : cmspi_info->io_def[k]);
			} else {
				/* else use set/clear registers */
				retval = target_write_u32(target, cmspi_info->io[k].addr
					+ (out ? dir_out_off : dir_inp_off), cmspi_info->io[k].mask);
			}
			if (retval != ERROR_OK) {
				LOG_ERROR("setting io[%d] direction failed", k);
				return retval;
			}
		}
	} while (k-- > 0);

	return retval;
}

FLASH_BANK_COMMAND_HANDLER(cmspi_flash_bank_command)
{
	struct cmspi_flash_bank *cmspi_info;
	uint16_t temp;
	uint8_t byte, byte_inp, byte_out;
	unsigned int index = 6;
	unsigned int j, k;

	LOG_DEBUG("%s", __func__);

	if (index + 5 > CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;

	cmspi_info = malloc(sizeof(struct cmspi_flash_bank));
	if (cmspi_info == NULL) {
		LOG_ERROR("Not enough memory");
		return ERROR_FAIL;
	}

	memset(cmspi_info, 0, sizeof(struct cmspi_flash_bank));
	bank->driver_priv = cmspi_info;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[index++], temp);
	if ((temp & ~(0xFFU << 2)) != 0) {
		command_print(CMD, "cmspi: inp offset must be 10-bit, word aligned");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	cmspi_info->inp_off = temp >> 2;		/* convert byte offset to word offset */

	if (index + 10 >= CMD_ARGC) {
		/* I2C mode has only 10 further parameters */
		LOG_INFO("I2C mode selected");
		cmspi_info->mode = MODE_I2C;

		/* two data pins to follow */
		k = 2;
	} else {
		LOG_INFO("SPI/DPI/QPI mode selected");
		cmspi_info->mode = MODE_SPI;		/* default mode */

		/* handle NCS and CLK pins */
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], cmspi_info->ncs.addr);
		if (cmspi_info->ncs.addr & 0x3) {
			command_print(CMD, "cmspi: NCS addr must be word aligned");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], byte);
		if (byte < 32) {
			cmspi_info->ncs.mask = 1UL << byte;
		} else {
			command_print(CMD, "cmspi: NCS bit number must be in 0 ... 31");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], cmspi_info->clk.addr);
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], byte);
		if (byte < 32) {
			cmspi_info->clk.mask = 1UL << byte;
		} else {
			command_print(CMD, "cmspi: CLK bit number must be in 0 ... 31");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (cmspi_info->clk.addr & 0x3) {
			command_print(CMD, "cmspi: CLK addr must be word aligned");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (index + 10 >= CMD_ARGC) {
			/* only two data pins following */
			k = 2;
		} else {
			/* all four data pins to follow */
			k = DATA_PINS;
		}
	}

	k -= 1;
	do {
		if (index + 5 > CMD_ARGC)
			return ERROR_COMMAND_SYNTAX_ERROR;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], cmspi_info->io[k].addr);
		if ((cmspi_info->io[k].addr == 0) || (cmspi_info->io[k].addr & 0x3)) {
			command_print(CMD, "cmspi: IO[%d] addr must be non-zero, word aligned", k);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], byte);
		if (byte < 32) {
			cmspi_info->bit_no |= (byte << (k << 3));
		} else {
			command_print(CMD, "cmspi: IO[%d] bit number must be in 0 ... 31", k);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[index++], temp);
		if (((temp & ~(0xFFU << 2)) != 0) || (temp == 0)) {
			command_print(CMD, "cmspi: dir_inp_off[%d] must be non-zero 10-bit, word aligned", k);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		cmspi_info->dir_inp_off |= (((temp >> 2) & 0xFF) << (k << 3));

		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[index++], temp);
		if (((temp & ~(0xFFU << 2)) != 0) || (temp == 0)) {
			command_print(CMD, "cmspi: dir_out_off[%d] must be non-zero 10-bit, word aligned", k);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		cmspi_info->dir_out_off |= (((temp >> 2) & 0xFF) << (k << 3));

		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], cmspi_info->io[k].mask);
	} while (k-- > 0);

	if (cmspi_info->mode == MODE_I2C) {
		/* copy IO1 data to NCS and IO0 to CLK */
		cmspi_info->SPI_SDA.addr = cmspi_info->io[I2C_SDA].addr;
		cmspi_info->SPI_SDA.mask = (1UL << ((cmspi_info->bit_no >> (I2C_SDA << 3)) & 0xFF));
		cmspi_info->SPI_SCL.addr = cmspi_info->io[I2C_SCL].addr;
		cmspi_info->SPI_SCL.mask = (1UL << ((cmspi_info->bit_no >> (I2C_SCL << 3)) & 0xFF));
		cmspi_info->same_port = (cmspi_info->io[I2C_SDA].addr == cmspi_info->io[I2C_SCL].addr);
		cmspi_info->same_conf =  (cmspi_info->same_port &&
			(((cmspi_info->dir_inp_off ^ (cmspi_info->dir_inp_off >> 8)) & 0xFF) == 0));

		LOG_DEBUG("SDA (out): 0x%08" PRIx32 ", (in) 0x%08" PRIx32
			", (dir_inp): 0x%08" PRIx32 ", (dir_out): 0x%08" PRIx32
			", (mask) 0x%08" PRIx32, cmspi_info->io[I2C_SDA].addr,
			cmspi_info->io[I2C_SDA].addr + INPUT_OFF,
			cmspi_info->io[I2C_SDA].addr + DIR_INP_OFF(I2C_SDA),
			cmspi_info->io[I2C_SDA].addr + DIR_OUT_OFF(I2C_SDA),
			cmspi_info->io[I2C_SDA].mask);
		LOG_DEBUG("SCL (out): 0x%08" PRIx32 ", (in) 0x%08" PRIx32
			", (dir_inp): 0x%08" PRIx32 ", (dir_out): 0x%08" PRIx32
			", (mask) 0x%08" PRIx32, cmspi_info->io[I2C_SCL].addr,
			cmspi_info->io[I2C_SCL].addr + INPUT_OFF,
			cmspi_info->io[I2C_SCL].addr + DIR_INP_OFF(I2C_SCL),
			cmspi_info->io[I2C_SCL].addr + DIR_OUT_OFF(I2C_SCL),
			cmspi_info->io[I2C_SCL].mask);
		if (cmspi_info->same_port)
			LOG_DEBUG("identical SDA/SCL input/output registers");
		if (cmspi_info->same_conf)
			LOG_DEBUG("identical SDA/SCL direction registers");
	} else {
		/* check for same port addresses and combine info accordingly */
		for (k = 0; k < DATA_PINS; k++) {
			byte_inp = (cmspi_info->dir_inp_off >> (k << 3)) & 0xFF;
			byte_out = (cmspi_info->dir_out_off >> (k << 3)) & 0xFF;
			for (j = k + 1; j < DATA_PINS; j++)
				if (cmspi_info->io[k].addr == cmspi_info->io[j].addr) {
					/* same port for pin j and pin k */
					if ((((cmspi_info->dir_inp_off >> (j << 3)) & 0xFF) == byte_inp) &&
						(((cmspi_info->dir_out_off >> (j << 3)) & 0xFF) == byte_out)) {
						/* same port and same dir offsets, then absorb dir info of
						* pin j into dir info for pin k */
						cmspi_info->io[k].mask |= cmspi_info->io[j].mask;

						/* clear dir info for pin j */
						cmspi_info->dir_inp_off &= ~(0xFFUL << (j << 3));
						cmspi_info->dir_out_off &= ~(0xFFUL << (j << 3));
						cmspi_info->io[j].mask = 0x0;
					}
				}
		}

		LOG_DEBUG("NCS (out): 0x%08" PRIx32 ", (mask): 0x%08" PRIx32,
			cmspi_info->ncs.addr, cmspi_info->ncs.mask);
		LOG_DEBUG("CLK (out): 0x%08" PRIx32 ", (mask): 0x%08" PRIx32,
			cmspi_info->clk.addr, cmspi_info->clk.mask);
		k = DATA_PINS - 1;
		do {
			if (cmspi_info->io[k].addr)
				LOG_DEBUG("IO%d (out): 0x%08" PRIx32 ", (in) 0x%08" PRIx32
					", (dir_inp): 0x%08" PRIx32 ", (dir_out): 0x%08" PRIx32
					", (mask) 0x%08" PRIx32, k, cmspi_info->io[k].addr,
					cmspi_info->io[k].addr + INPUT_OFF,
					cmspi_info->io[k].addr + DIR_INP_OFF(k),
					cmspi_info->io[k].addr + DIR_OUT_OFF(k),
					cmspi_info->io[k].mask);
		} while (k-- > 0);
	}

	LOG_DEBUG("bit_no:      0x%08" PRIx32, cmspi_info->bit_no);
	LOG_DEBUG("inp_off:     0x%08" PRIx32, (int8_t) cmspi_info->inp_off);
	LOG_DEBUG("dir_inp_off: 0x%08" PRIx32, cmspi_info->dir_inp_off);
	LOG_DEBUG("dir_out_off: 0x%08" PRIx32, cmspi_info->dir_out_off);

	return ERROR_OK;
}

/* send start or stop condition */
static int cmspi_i2c_start_stop(struct flash_bank *bank, bool start)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port_clk, pdir_clk, port_iox, pdir_iox;
	int retval;

	retval = target_read_u32(target, cmspi_info->SPI_SCL.addr, &port_clk);
	if (retval != ERROR_OK)
		return retval;

	if (cmspi_info->same_port)
		port_iox = port_clk;
	else {
		retval = target_read_u32(target, cmspi_info->SPI_SDA.addr, &port_iox);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cmspi_info->dir_inp_off == cmspi_info->dir_out_off) {
		/* if read-modify-write direction register, get current dir setting */
		retval = target_read_u32(target, cmspi_info->SPI_SCL.addr
			+ DIR_INP_OFF(I2C_SCL), &pdir_clk);
		if (retval != ERROR_OK)
			return retval;

		if (cmspi_info->same_conf)
			pdir_iox = pdir_clk;
		else {
			retval = target_read_u32(target, cmspi_info->SPI_SDA.addr
				+ DIR_INP_OFF(I2C_SDA), &pdir_iox);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	if (!start) {
		CLR_SDA(port_iox, pdir_iox)
		if (retval != ERROR_OK)
			return retval;

		/* update port_clk data if modified */
		if (cmspi_info->same_port)
			port_clk = port_iox;
		if (cmspi_info->same_conf)
			pdir_clk = pdir_iox;
	}

	/* set SCL */
	SET_SCL(port_clk, pdir_clk);
	if (retval != ERROR_OK)
		return retval;

	/* update port_iox data if modified */
	if (cmspi_info->same_port)
		port_iox = port_clk;
	if (cmspi_info->same_conf)
		pdir_iox = pdir_clk;

	/* clear/set SDA */
	if (start)
		CLR_SDA(port_iox, pdir_iox)
	else
		SET_SDA(port_iox, pdir_iox)
	if (retval != ERROR_OK)
		return retval;

	if (start) {
		/* update port_iox data if modified */
		if (cmspi_info->same_port)
			port_clk = port_iox;
		if (cmspi_info->same_conf)
			pdir_clk = pdir_iox;

		/* clear SCL */
		CLR_SCL(port_clk, pdir_clk);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/* reset I2C communication if synchronization is lost */
static int cmspi_i2c_resync(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port_clk, pdir_clk, port_iox, pdir_iox;
	int k, retval;

	/* reset SCL to proper '1' level */
	retval = target_read_u32(target, cmspi_info->SPI_SCL.addr, &port_clk);
	if (retval != ERROR_OK)
		return retval;
	port_clk |= (cmspi_info->SPI_SCL.mask);
	pdir_clk = cmspi_info->io_def[I2C_SCL];
	SET_SCL(port_clk, pdir_clk);

	if (cmspi_info->same_port)
		port_iox = port_clk;
	else {
		retval = target_read_u32(target, cmspi_info->SPI_SDA.addr, &port_iox);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cmspi_info->dir_inp_off == cmspi_info->dir_out_off) {
		/* if read-modify-write direction register, get current dir setting */
		retval = target_read_u32(target, cmspi_info->SPI_SCL.addr
			+ DIR_INP_OFF(I2C_SCL), &pdir_clk);
		if (retval != ERROR_OK)
			return retval;

		if (cmspi_info->same_conf)
			pdir_iox = pdir_clk;
		else {
			retval = target_read_u32(target, cmspi_info->SPI_SDA.addr
				+ DIR_INP_OFF(I2C_SDA), &pdir_iox);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* reset SDA to proper '1' level */
	port_iox |= (cmspi_info->SPI_SDA.mask);
	pdir_iox = cmspi_info->io_def[I2C_SDA];
	SET_SDA(port_iox, pdir_iox);

	/* update port_clk data if modified */
	if (cmspi_info->same_port)
		port_clk = port_iox;
	if (cmspi_info->same_conf)
		pdir_clk = pdir_iox;

	/* generate start condition 9 times */
	for (k = 0; k < 9; k++) {
		SET_SDA(port_iox, pdir_iox);
		/* update port_clk data if modified */
		if (cmspi_info->same_port)
			port_clk = port_iox;
		if (cmspi_info->same_conf)
			pdir_clk = pdir_iox;

		SET_SCL(port_clk, pdir_clk);
		/* update port_iox data if modified */
		if (cmspi_info->same_port)
			port_iox = port_clk;
		if (cmspi_info->same_conf)
			pdir_iox = pdir_clk;

		CLR_SDA(port_iox, pdir_iox);
		/* update port_clk data if modified */
		if (cmspi_info->same_port)
			port_clk = port_iox;
		if (cmspi_info->same_conf)
			pdir_clk = pdir_iox;

		CLR_SCL(port_clk, pdir_clk);
		/* update port_iox data if modified */
		if (cmspi_info->same_port)
			port_iox = port_clk;
		if (cmspi_info->same_conf)
			pdir_iox = pdir_clk;
	}

	/* and finally one stop condition */
	SET_SCL(port_clk, pdir_clk);
	/* update port_iox data if modified */
	if (cmspi_info->same_port)
		port_iox = port_clk;
	if (cmspi_info->same_conf)
		pdir_iox = pdir_clk;

	SET_SDA(port_iox, pdir_iox);
	/* update port_clk data if modified */
	if (cmspi_info->same_port)
		port_clk = port_iox;
	if (cmspi_info->same_conf)
		pdir_clk = pdir_iox;

	return retval;
}

/* Send and receive one byte via SPI/DPI/QPI/I2C */
/* bits 7 down to 0 are shifted out, MSB first */
static int cmspi_shift_out(struct flash_bank *bank, uint32_t word)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port_clk, pdir_clk, port_iox, pdir_iox, in_iox, bit_mask;
	int k, m, retval;

	LOG_DEBUG("0x%02" PRIx8, word);

	retval = target_read_u32(target, cmspi_info->clk.addr, &port_clk);
	if (retval != ERROR_OK)
		return retval;

	switch (cmspi_info->mode) {
		case MODE_SPI:
			if (cmspi_info->clk.addr == cmspi_info->io[0].addr)
				port_iox = port_clk;
			else {
				retval = target_read_u32(target, cmspi_info->io[0].addr, &port_iox);
				if (retval != ERROR_OK)
					return retval;
			}

			bit_mask = 1UL << ((cmspi_info->bit_no >> 0) & 0xFF);
			for (k = 0; k < 8; k++)	{
				/* copy current data bit to port data */
				if (word & (1UL << 7))
					port_iox |= bit_mask;
				else
					port_iox &= ~bit_mask;
				/* shift next bit into bit 7 */
				word <<= 1;

				retval = target_write_u32(target, cmspi_info->io[0].addr, port_iox);
				if (retval != ERROR_OK)
					return retval;

				/* update port_clk data if modified */
				if (cmspi_info->clk.addr == cmspi_info->io[0].addr)
					port_clk = port_iox;

				/* set CLK */
				SET_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;

				/* clear CLK */
				CLR_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;
			}
			break;

		case MODE_DPI:
			for (k = 0; k < 8 / 2; k++) {
				/* shift out all four pairs, msb first */
				for (m = 2 - 1; m >= 0; --m) {
					if (cmspi_info->clk.addr == cmspi_info->io[m].addr)
						port_iox = port_clk;
					else {
						retval = target_read_u32(target, cmspi_info->io[m].addr, &port_iox);
						if (retval != ERROR_OK)
							return retval;
					}

					/* copy current data bit to port data */
					bit_mask =  1UL << ((cmspi_info->bit_no >> (m << 3)) & 0xFF);
					if (word & (1UL << 7))
						port_iox |= bit_mask;
					else
						port_iox &= ~bit_mask;
					/* shift next bit into bit 7 */
					word <<= 1;

					retval = target_write_u32(target, cmspi_info->io[m].addr, port_iox);
					if (retval != ERROR_OK)
						return retval;

					/* update port_clk data if modified */
					if (cmspi_info->clk.addr == cmspi_info->io[m].addr)
						port_clk = port_iox;
				}

				/* set CLK */
				SET_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;

				/* clear CLK */
				CLR_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;
			}
			break;

		case MODE_QPI:
			for (k = 0; k < 8 / DATA_PINS; k++) {
				/* shift out both nibbles, msb first */
				for (m = DATA_PINS - 1; m >= 0; --m) {
					if (cmspi_info->clk.addr == cmspi_info->io[m].addr)
						port_iox = port_clk;
					else {
						retval = target_read_u32(target, cmspi_info->io[m].addr, &port_iox);
						if (retval != ERROR_OK)
							return retval;
					}

					/* copy current data bit to port data */
					bit_mask =  1UL << ((cmspi_info->bit_no >> (m << 3)) & 0xFF);
					if (word & (1UL << 7))
						port_iox |= bit_mask;
					else
						port_iox &= ~bit_mask;
					/* shift next bit into bit 7 */
					word <<= 1;

					retval = target_write_u32(target, cmspi_info->io[m].addr, port_iox);
					if (retval != ERROR_OK)
						return retval;

					/* update port_clk data if modified */
					if (cmspi_info->clk.addr == cmspi_info->io[m].addr)
						port_clk = port_iox;
				}

				/* set CLK */
				SET_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;

				/* clear CLK */
				CLR_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;
			}
			break;

		case MODE_I2C:
			if (cmspi_info->dir_inp_off == cmspi_info->dir_out_off) {
				/* if read-modify-write direction register, get current dir */
				retval = target_read_u32(target, cmspi_info->SPI_SCL.addr
					+ DIR_INP_OFF(I2C_SCL), &pdir_clk);
				if (retval != ERROR_OK)
					return retval;

				if (cmspi_info->same_conf)
					pdir_iox = pdir_clk;
				else {
					retval = target_read_u32(target, cmspi_info->SPI_SDA.addr
						+ DIR_INP_OFF(I2C_SDA), &pdir_iox);
					if (retval != ERROR_OK)
						return retval;
				}
			}

			if (cmspi_info->same_port)
				port_iox = port_clk;
			else {
				retval = target_read_u32(target, cmspi_info->SPI_SDA.addr, &port_iox);
				if (retval != ERROR_OK)
					return retval;
			}

			for (k = 0; k < 8; k++)	{
				/* output SDA value */
				if (word & (1UL << 7))
					SET_SDA(port_iox, pdir_iox)
				else
					CLR_SDA(port_iox, pdir_iox)
				if (retval != ERROR_OK)
					return retval;

				/* update port_clk data if modified */
				if (cmspi_info->same_port)
					port_clk = port_iox;
				if (cmspi_info->same_conf)
					pdir_clk = pdir_iox;

				/* set SCL */
				SET_SCL(port_clk, pdir_clk);
				if (retval != ERROR_OK)
					return retval;

				/* shift next bit into bit 7 */
				word <<= 1;

				/* clear SCL */
				CLR_SCL(port_clk, pdir_clk);
				if (retval != ERROR_OK)
					return retval;
			}

			/* now ACK/NACK follows: release SDA */
			SET_SDA(port_iox, pdir_iox);
			if (retval != ERROR_OK)
				return retval;

			/* update port_clk data if modified */
			if (cmspi_info->same_port)
				port_clk = port_iox;
			if (cmspi_info->same_conf)
				pdir_clk = pdir_iox;

			/* set SCL */
			SET_SCL(port_clk, pdir_clk);
			if (retval != ERROR_OK)
				return retval;

			/* check SDA for ACK/NACK */
			retval = target_read_u32(target,
				cmspi_info->SPI_SDA.addr + INPUT_OFF, &in_iox);
			if (retval != ERROR_OK)
				return retval;

			/* clear SCL */
			CLR_SCL(port_clk, pdir_clk);
			if (retval != ERROR_OK)
				return retval;

			LOG_DEBUG("I2C %s", in_iox & cmspi_info->ncs.mask ? "NACK" : "ACK");
			/* return error if NACK */
			if (in_iox & cmspi_info->SPI_SDA.mask)
				return ERROR_FLASH_OPERATION_FAILED;

			break;

		default:
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* Receive one byte via SPI/DPI/QPI/I2C */
/* bits 7 down to 0 are shifted in, MSB first */
/* For I2C only: ACK is generated only if bit 0 of word was 1 */
static int cmspi_shift_in(struct flash_bank *bank, uint32_t *word)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port_clk, pdir_clk, port_iox, pdir_iox;
	uint8_t bit_no;
	int k, m, retval;

	switch (cmspi_info->mode) {
		case MODE_SPI:
			retval = target_read_u32(target, cmspi_info->clk.addr, &port_clk);
			if (retval != ERROR_OK)
				return retval;

			bit_no = (cmspi_info->bit_no >> (1 << 3)) & 0xFF;
			for (k = 0; k < 8; k++)	{
				/* shift in data bits, msb first */
				/* set CLK */
				SET_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;

				/* read data bit */
				retval = target_read_u32(target,
					cmspi_info->io[1].addr + INPUT_OFF, &port_iox);
				if (retval != ERROR_OK)
					return retval;

				*word <<= 1;
				*word |= ((port_iox >> bit_no) & 0x1);

				/* clear CLK */
				CLR_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;
			}
			break;

		case MODE_DPI:
			retval = target_read_u32(target, cmspi_info->clk.addr, &port_clk);
			if (retval != ERROR_OK)
				return retval;

			for (k = 0; k < 8 / 2; k++) {
				/* shift in all four pairs, msb first */

				/* set CLK */
				SET_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;

				for (m = 2 - 1; m >= 0; --m) {
					/* shift in data bits, msb first */
					retval = target_read_u32(target,
						cmspi_info->io[m].addr + INPUT_OFF, &port_iox);
					if (retval != ERROR_OK)
						return retval;

					bit_no = (cmspi_info->bit_no >> (m << 3)) & 0xFF;
					*word <<= 1;
					*word |= ((port_iox >> bit_no) & 0x1);
				}

				/* clear CLK */
				CLR_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;
			}
			break;

		case MODE_QPI:
			retval = target_read_u32(target, cmspi_info->clk.addr, &port_clk);
			if (retval != ERROR_OK)
				return retval;

			for (k = 0; k < 8 / DATA_PINS; k++) {
				/* shift in both nibbles, msb first */

				/* set CLK */
				SET_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;

				for (m = DATA_PINS - 1; m >= 0; --m) {
					/* shift in data bits, msb first */
					retval = target_read_u32(target,
						cmspi_info->io[m].addr + INPUT_OFF, &port_iox);
					if (retval != ERROR_OK)
						return retval;

					bit_no = (cmspi_info->bit_no >> (m << 3)) & 0xFF;
					*word <<= 1;
					*word |= ((port_iox >> bit_no) & 0x1);
				}

				/* clear CLK */
				CLR_PORT_BIT(port_clk, clk);
				if (retval != ERROR_OK)
					return retval;
			}
			break;

		case MODE_I2C:
			retval = target_read_u32(target, cmspi_info->SPI_SCL.addr, &port_clk);
			if (retval != ERROR_OK)
				return retval;

			if (cmspi_info->dir_inp_off == cmspi_info->dir_out_off) {
				/* if read-modify-write direction register, get current dir */
				retval = target_read_u32(target, cmspi_info->SPI_SCL.addr
					+ DIR_INP_OFF(I2C_SCL), &pdir_clk);
				if (retval != ERROR_OK)
					return retval;
			}

			for (k = 0; k < 8; k++)	{
				/* set SCL */
				SET_SCL(port_clk, pdir_clk);
				if (retval != ERROR_OK)
					return retval;

				/* read SDA */
				retval = target_read_u32(target, cmspi_info->SPI_SDA.addr + INPUT_OFF, &port_iox);
				if (retval != ERROR_OK)
					return retval;

				*word = (*word << 1) | ((port_iox & cmspi_info->SPI_SDA.mask) ? 0x1 : 0x0);

				/* clear SCL */
				CLR_SCL(port_clk, pdir_clk);
				if (retval != ERROR_OK)
					return retval;
			}

			/* retrieve port data */
			if (cmspi_info->same_port)
				port_iox = port_clk;
			else {
				retval = target_read_u32(target, cmspi_info->SPI_SDA.addr, &port_iox);
				if (retval != ERROR_OK)
					return retval;
			}

			if (cmspi_info->dir_inp_off == cmspi_info->dir_out_off) {
				/* if read-modify-write direction register, get current dir */
				if (cmspi_info->same_conf)
					pdir_iox = pdir_clk;
				else {
					retval = target_read_u32(target, cmspi_info->SPI_SDA.addr
						+ DIR_INP_OFF(I2C_SDA), &pdir_iox);
					if (retval != ERROR_OK)
						return retval;
				}
			}

			if (*word & 0x100) {
				/* now send ACK if requested: pull SDA low */
				LOG_DEBUG("send ACK");
				CLR_SDA(port_iox, pdir_iox);
				if (retval != ERROR_OK)
					return retval;
			} else {
				/* now send NACK if requested: keep SDA high */
				LOG_DEBUG("send NACK");
				SET_SDA(port_iox, pdir_iox);
				if (retval != ERROR_OK)
					return retval;
			}

			/* update port_clk data if modified */
			if (cmspi_info->same_port)
				port_clk = port_iox;
			if (cmspi_info->same_conf)
				pdir_clk = pdir_iox;

			/* set SCL */
			SET_SCL(port_clk, pdir_clk);
			if (retval != ERROR_OK)
				return retval;

			/* clear SCL */
			CLR_SCL(port_clk, pdir_clk);
			if (retval != ERROR_OK)
				return retval;

			/* release SDA if asserted */
			SET_SDA(port_iox, pdir_iox);
			if (retval != ERROR_OK)
				return retval;
			break;

		default:
			return ERROR_FAIL;
	}

	LOG_DEBUG("0x%02" PRIx8, *word);

	return ERROR_OK;
}

/* Read status register of the external SPI flash chip. */
static int read_status_reg(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port;
	int retval, success;

	/* invalidate status */
	*status = 0xFF;

	success = ERROR_FAIL;

	/* set NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		goto err;
	SET_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		goto err;

	/* clear NCS */
	CLR_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		goto err;

	/* set to output */
	retval = set_to_output(bank, true);
	if (retval != ERROR_OK)
		goto err;

	/* send command byte */
	retval = cmspi_shift_out(bank, SPIFLASH_READ_STATUS);
	if (retval != ERROR_OK)
		goto err;

	/* set to input */
	retval = set_to_output(bank, false);
	if (retval != ERROR_OK)
		goto err;

	/* get result byte */
	success = cmspi_shift_in(bank, status);
	*status &= 0xFF;

err:
	/* set to input, ignoring errors */
	set_to_output(bank, false);

	/* set NCS, ignoring errors */
	target_read_u32(target, cmspi_info->ncs.addr, &port);
	SET_PORT_BIT(port, ncs);

	/* set to input */
	set_to_output(bank, false);

	return success;
}

/* Check for WIP (write in progress) bit in status register */
/* timeout in ms */
static int wait_till_ready(struct flash_bank *bank, int timeout)
{
	uint32_t status;
	int retval;
	long long endtime;

	endtime = timeval_ms() + timeout;
	do {
		/* Read flash status register */
		retval = read_status_reg(bank, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & SPIFLASH_BSY_BIT) == 0)
			return ERROR_OK;
		alive_sleep(25);
	} while (timeval_ms() < endtime);

	LOG_WARNING("timeout");
	return ERROR_FLASH_BUSY;
}

/* Send "write enable" command to SPI flash chip */
static int cmspi_write_enable(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port, status;
	int retval;

	/* clear NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		goto err;
	CLR_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		goto err;

	/* set to output */
	retval = set_to_output(bank, true);
	if (retval != ERROR_OK)
		goto err;

	/* send write enable command */
	retval = cmspi_shift_out(bank, SPIFLASH_WRITE_ENABLE);
	if (retval != ERROR_OK)
		goto err;

err:
	/* set to input, ignoring errors */
	set_to_output(bank, false);

	/* set NCS */
	SET_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		return retval;

	/* Read flash status register, sets to input */
	retval = read_status_reg(bank, &status);
	if (retval != ERROR_OK)
		return retval;

	/* check write enabled */
	if ((status & SPIFLASH_WE_BIT) == 0) {
		LOG_ERROR("Cannot enable write to flash. Status=0x%02" PRIx8, status);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* Erase a single sector */
static int cmspi_erase_sector(struct flash_bank *bank, int sector)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t addr = bank->sectors[sector].offset;
	uint32_t port, data;
	int retval;

	retval = cmspi_write_enable(bank);
	if (retval != ERROR_OK)
		return retval;

	if (bank->sectors[sector].is_protected) {
		LOG_ERROR("Flash sector %d protected", sector);
		return ERROR_FLASH_PROTECTED;
	} else
		bank->sectors[sector].is_erased = -1;

	/* Send Sector Erase command */
	/* clear NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		goto err;
	CLR_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		goto err;

	/* set to output */
	retval = set_to_output(bank, true);
	if (retval != ERROR_OK)
		goto err;

	/* send command byte */
	retval = cmspi_shift_out(bank, cmspi_info->dev.erase_cmd);
	if (retval != ERROR_OK)
		goto err;

	if (cmspi_info->dev.size_in_bytes > (1UL << 24)) {
		/* bits 24-31 */
		retval = cmspi_shift_out(bank, addr >> 24);
		if (retval != ERROR_OK)
			goto err;
	}

	if (cmspi_info->dev.size_in_bytes > (1UL << 16)) {
		/* bits 16-23 */
		retval = cmspi_shift_out(bank, addr >> 16);
		if (retval != ERROR_OK)
			goto err;
	}

	if (cmspi_info->dev.size_in_bytes > (1UL << 8)) {
		/* bits 8-15 */
		retval = cmspi_shift_out(bank, addr >> 8);
		if (retval != ERROR_OK)
			goto err;
	}

	/* bits 0-7 */
	retval = cmspi_shift_out(bank, addr >> 0);
	if (retval != ERROR_OK)
		goto err;

err:
	/* set to input, ignoring errors */
	set_to_output(bank, false);

	/* set NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		return retval;

	SET_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		return retval;

	/* read flash status register */
	retval = read_status_reg(bank, &data);
	if (retval != ERROR_OK)
		return retval;

	/* check for command in progress for flash */
	if (((data & SPIFLASH_BSY_BIT) == 0) && (data & SPIFLASH_WE_BIT) != 0) {
		LOG_DEBUG("Sector erase not accepted by flash. Status=0x%02" PRIx8,
			data & 0xFF);
		/* return ERROR_FAIL; */
	}

	/* erasure takes a long time, so some sort of progress message is a good idea */
	LOG_DEBUG("erasing sector %4d", sector);

	/* poll WIP for end of self timed Sector Erase cycle */
	retval = wait_till_ready(bank, CMSPI_MAX_TIMEOUT);

	return retval;
}

/* Erase range of sectors */
static int cmspi_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	int retval = ERROR_OK;
	int sector;

	LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(cmspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (cmspi_info->dev.erase_cmd == 0x00) {
		LOG_ERROR("Sector erase not available");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FLASH_PROTECTED;
		}
	}

	for (sector = first; sector <= last; sector++) {
		retval = cmspi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		keep_alive();
	}

	if (retval != ERROR_OK)
		LOG_ERROR("Flash sector_erase failed on sector %d", sector);

	return retval;
}

/* Delay calibration for I2C */
int cmspi_i2c_calib(struct flash_bank *bank, uint32_t count, double *time)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	struct duration bench;
	struct reg_param reg_params[1];
	struct armv7m_algorithm armv7m_info;
	struct working_area *calib_algorithm;
	uint32_t maxsize, exit_point;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* see contrib/loaders/erase_check/cmi2c_calib.S for src */
	static const uint8_t cmi2c_calib_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmi2c_calib.inc"
	};
	const uint32_t codesize = sizeof(cmi2c_calib_code);

	maxsize = target_get_working_area_avail(target);
	if (maxsize < codesize) {
		LOG_WARNING("Not enough working area, can't do I2C calibration");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (target_alloc_working_area_try(target, codesize,
			&calib_algorithm) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* this will overlay the words of cm*_*_code in target */
	uint32_t port_buffer[] = FILL_PORT_BUFFER(0x00);

	/* prepare check code, excluding port_buffer */
	retval = target_write_buffer(target, calib_algorithm->address,
		codesize - sizeof(port_buffer), cmi2c_calib_code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare port_buffer values */
	retval = target_write_buffer(target, calib_algorithm->address
		+ codesize - sizeof(port_buffer),
		sizeof(port_buffer), (uint8_t *) port_buffer);
	if (retval != ERROR_OK)
		goto err;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);	/* loop counter */
	buf_set_u32(reg_params[0].value, 0, 32, count);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* after breakpoint instruction (halfword) one nop (halfword) till end of code */
	exit_point = calib_algorithm->address + codesize
		- sizeof(uint32_t) - sizeof(port_buffer);

	duration_start(&bench);

	retval = target_run_algorithm(target,
		0, NULL,
		ARRAY_SIZE(reg_params), reg_params,
		calib_algorithm->address, exit_point,
		10000,
		&armv7m_info);
	if (retval != ERROR_OK)
		goto err;

	duration_measure(&bench);
	keep_alive();
	destroy_reg_param(&reg_params[0]);

	LOG_DEBUG("%f ms", duration_elapsed(&bench) * 1.0E3);
	*time = duration_elapsed(&bench);
err:
	target_free_working_area(target, calib_algorithm);

	return retval;
}

/* Scan I2C bus for slaves */
int cmspi_i2c_scan(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	struct armv7m_algorithm armv7m_info;
	struct working_area *scan_algorithm;
	uint32_t maxsize, exit_point, scan_info[SCAN_SIZE];
	uint16_t addr;
	const int max = 16;
	char temp[4], output[max * 3 + 1];
	int count, retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* see contrib/loaders/flash/cmspi/cmcmi2c_scan.S for src */
	static const uint8_t cmi2c_scan_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmi2c_scan.inc"
	};
	const uint32_t codesize = sizeof(cmi2c_scan_code);

	maxsize = target_get_working_area_avail(target);
	if (maxsize < codesize + SCAN_SIZE) {
		LOG_WARNING("Not enough working area, can't do I2C scan");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (target_alloc_working_area_try(target, codesize
		+ SCAN_SIZE * sizeof(scan_info[0]), &scan_algorithm) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* this will overlay the words of cm*_*_code in target */
	uint32_t port_buffer[] = FILL_PORT_BUFFER(0x00);

	/* prepare check code, excluding port_buffer */
	retval = target_write_buffer(target, scan_algorithm->address,
		codesize - sizeof(port_buffer), cmi2c_scan_code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare port_buffer values */
	retval = target_write_buffer(target, scan_algorithm->address
		+ codesize - sizeof(port_buffer),
		sizeof(port_buffer), (uint8_t *) port_buffer);
	if (retval != ERROR_OK)
		goto err;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* after breakpoint instruction (halfword) one nop (halfword) and
	 * port_buffer till end of code */
	exit_point = scan_algorithm->address + codesize
		- sizeof(uint32_t) - sizeof(port_buffer);

	retval = target_run_algorithm(target,
		0, NULL,
		0, NULL,
		scan_algorithm->address, exit_point,
		10000,
		&armv7m_info);
	if (retval != ERROR_OK)
		goto err;

	keep_alive();

	retval = target_read_buffer(target, scan_algorithm->address + codesize,
			SCAN_SIZE * sizeof(scan_info[0]), (uint8_t *) &scan_info);
	if (retval != ERROR_OK)
		goto err;

	/* print the scan result */
	count = 0;
	strncpy(output, "", 1);
	for (addr = I2C_FIRST; addr <= I2C_LAST; addr++) {
		if (!(scan_info[(addr - I2C_FIRST) >> 5] &
			(1UL << (31 - ((addr - I2C_FIRST) & ((1UL << 5) - 1)))))) {
			snprintf(temp, sizeof(temp), "%02" PRIx8 " ", addr);
			strncat(output, temp, sizeof(output) - strlen(output) - 1);
			if (++count >= max) {
				LOG_INFO("i2c_scan: %s", output);
				count = 0;
				strncpy(output, "", 1);
			}
		}
	}
	if (count != 0)
		LOG_INFO("i2c_scan: %s", output);

err:
	target_free_working_area(target, scan_algorithm);

	return retval;
}

/* Check whether flash is blank */
static int cmspi_blank_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	struct duration bench;
	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;
	struct working_area *algorithm;
	static const uint8_t *code;
	struct sector_info erase_check_info;
	uint32_t codesize, maxsize, exit_point, result, port;
	uint8_t cmd;
	int num_sectors, sector, index, count, retval;
	const uint32_t erased = 0x00FF;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(cmspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* see contrib/loaders/flash/cmspi/cmspi_erase_check.S for src */
	static const uint8_t cmspi_erase_check_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmspi_erase_check.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmdpi_erase_check.S for src */
	static const uint8_t cmdpi_erase_check_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmdpi_erase_check.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmqpi_erase_check.S for src */
	static const uint8_t cmqpi_erase_check_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmqpi_erase_check.inc"
	};

	/* see contrib/loaders/erase_check/cmi2c_erase_check.S for src */
	static const uint8_t cmi2c_erase_check_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmi2c_erase_check.inc"
	};

	switch (cmspi_info->mode) {
		case MODE_SPI:
			code = cmspi_erase_check_code;
			codesize = sizeof(cmspi_erase_check_code);
			cmd = cmspi_info->dev.read_cmd;
			break;

		case MODE_DPI:
			code = cmdpi_erase_check_code;
			codesize = sizeof(cmdpi_erase_check_code);
			cmd = cmspi_info->dev.qread_cmd;
			break;

		case MODE_QPI:
			code = cmqpi_erase_check_code;
			codesize = sizeof(cmqpi_erase_check_code);
			cmd = cmspi_info->dev.qread_cmd;
			break;

		case MODE_I2C:
			code = cmi2c_erase_check_code;
			codesize = sizeof(cmi2c_erase_check_code);
			cmd = cmspi_info->i2c_addr;
			break;

		default:
			return ERROR_FAIL;
	}

	/* this will overlay the last words of cm*_erase_check_code in target */
	uint32_t port_buffer[] = FILL_PORT_BUFFER(cmd);

	maxsize = target_get_working_area_avail(target);
	if (maxsize < codesize + sizeof(erase_check_info)) {
		LOG_WARNING("Not enough working area, can't do SPI blank check");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	num_sectors = (maxsize - codesize) / sizeof(erase_check_info);
	num_sectors = (bank->num_sectors < num_sectors) ? bank->num_sectors : num_sectors;

	if (target_alloc_working_area_try(target,
			codesize + num_sectors * sizeof(erase_check_info), &algorithm) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* prepare check code, excluding port_buffer */
	retval = target_write_buffer(target, algorithm->address,
		codesize - sizeof(port_buffer), code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare port_buffer values */
	retval = target_write_buffer(target, algorithm->address + codesize - sizeof(port_buffer),
		sizeof(port_buffer), (uint8_t *) port_buffer);
	if (retval != ERROR_OK)
		goto err;

	duration_start(&bench);

	/* after breakpoint instruction (halfword) one nop (halfword) and
	 * port_buffer till end of code */
	exit_point = algorithm->address + codesize - sizeof(uint32_t) - sizeof(port_buffer);

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);	/* sector count */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* flash pagesize */
	init_reg_param(&reg_params[2], "r3", 32, PARAM_OUT);	/* 1/2/3/4-byte address mode */

	sector = 0;
	while (sector < bank->num_sectors) {
		/* at most num_sectors sectors to handle in one run */
		count = bank->num_sectors - sector;
		if (count > num_sectors)
			count = num_sectors;

		for (index = 0; index < count; index++) {
			erase_check_info.offset = h_to_le_32(bank->sectors[sector + index].offset);
			erase_check_info.size = h_to_le_32(bank->sectors[sector + index].size);
			erase_check_info.result = h_to_le_32(erased);

			retval = target_write_buffer(target, algorithm->address
				+ codesize + index * sizeof(erase_check_info),
					sizeof(erase_check_info), (uint8_t *) &erase_check_info);
			if (retval != ERROR_OK)
				goto err;
		}

		buf_set_u32(reg_params[0].value, 0, 32, count);
		buf_set_u32(reg_params[1].value, 0, 32, cmspi_info->dev.sectorsize);
		buf_set_u32(reg_params[2].value, 0, 32,
			(cmspi_info->mode == MODE_I2C) ? I2C_ADDR_BYTES : SPI_ADDR_BYTES);

		armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
		armv7m_info.core_mode = ARM_MODE_THREAD;

		LOG_DEBUG("checking sectors %d to %d", sector, sector + count - 1);
		/* check a block of sectors */
		retval = target_run_algorithm(target,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			algorithm->address, exit_point,
			4 * count * (bank->sectors[sector].size >>
				((cmspi_info->mode == MODE_I2C) ? 0 : 6)),
			&armv7m_info);
		if (retval != ERROR_OK)
			break;

		for (index = 0; index < count; index++) {
			retval = target_read_buffer(target, algorithm->address
				+ codesize + index * sizeof(erase_check_info),
					sizeof(erase_check_info), (uint8_t *) &erase_check_info);
			if (retval != ERROR_OK)
				goto err;

			if ((erase_check_info.offset != h_to_le_32(bank->sectors[sector + index].offset)) ||
				(erase_check_info.size != 0)) {
				LOG_ERROR("corrupted blank check info");
				goto err;
			}

			/* we need le_32_to_h, but that's the same as h_to_le_32 */
			result = h_to_le_32(erase_check_info.result);
			bank->sectors[sector + index].is_erased = ((result & 0xFF) == 0xFF);
			LOG_DEBUG("Flash sector %d checked: %04x", sector + index, result & 0xFFFF);
		}
		keep_alive();
		sector += count;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	duration_measure(&bench);
	LOG_INFO("cmspi blank checked in %fs (%0.3f KiB/s)",
		duration_elapsed(&bench), duration_kbps(&bench, bank->size));

err:
	if (cmspi_info->mode == MODE_I2C) {
		/* error recovery */
		cmspi_i2c_resync(bank);
	} else {
		/* set to input */
		set_to_output(bank, false);

		/* set NCS, ignoring errors */
		target_read_u32(target, cmspi_info->ncs.addr, &port);
		port |= (cmspi_info->ncs.mask);
		target_write_u32(target, cmspi_info->ncs.addr, port);
	}

	target_free_working_area(target, algorithm);

	return retval;
}

/* Verify checksum */
static int cmspi_verify(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	struct working_area *algorithm;
	const uint8_t *code;
	uint8_t cmd;
	uint32_t codesize, crc32, port, result, exit_point;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(cmspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > cmspi_info->dev.size_in_bytes) {
		LOG_WARNING("Verify beyond end of flash. Extra data to be ignored.");
		count = cmspi_info->dev.size_in_bytes - offset;
	}

	/* see contrib/loaders/flash/cmspi/cmspi_verify.S for src */
	static const uint8_t cmspi_verify_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmspi_verify.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmdpi_verify.S for src */
	static const uint8_t cmdpi_verify_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmdpi_verify.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmqpi_verify.S for src */
	static const uint8_t cmqpi_verify_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmqpi_verify.inc"
	};

	/* see contrib/loaders/erase_check/cmi2c_verify.S for src */
	static const uint8_t cmi2c_verify_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmi2c_verify.inc"
	};

	switch (cmspi_info->mode) {
		case MODE_SPI:
			code = cmspi_verify_code;
			codesize = sizeof(cmspi_verify_code);
			cmd = cmspi_info->dev.read_cmd;
			break;

		case MODE_DPI:
			code = cmdpi_verify_code;
			codesize = sizeof(cmdpi_verify_code);
			cmd = cmspi_info->dev.qread_cmd;
			break;

		case MODE_QPI:
			code = cmqpi_verify_code;
			codesize = sizeof(cmqpi_verify_code);
			cmd = cmspi_info->dev.qread_cmd;
			break;

		case MODE_I2C:
			code = cmi2c_verify_code;
			codesize = sizeof(cmi2c_verify_code);
			cmd = cmspi_info->i2c_addr;
			break;

		default:
			return ERROR_FAIL;
	}

	/* this will overlay the last words of cm*_verify_code in target */
	uint32_t port_buffer[] = FILL_PORT_BUFFER(cmd);

	if (target_alloc_working_area_try(target, codesize, &algorithm) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* prepare verify code, excluding port_buffer */
	retval = target_write_buffer(target, algorithm->address,
		codesize - sizeof(port_buffer), code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare port_buffer values */
	retval = target_write_buffer(target, algorithm->address + codesize - sizeof(port_buffer),
		sizeof(port_buffer), (uint8_t *) port_buffer);
	if (retval != ERROR_OK)
		goto err;

	/* after breakpoint instruction (halfword) one nop (halfword) and
	 * port_buffer till end of code */
	exit_point = algorithm->address + codesize - sizeof(uint32_t) - sizeof(port_buffer);

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* count (in), crc32 (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* flash sectorsize */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);	/* offset into flash address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* 1/2/3/4-byte address mode */

	buf_set_u32(reg_params[0].value, 0, 32, count);
	buf_set_u32(reg_params[1].value, 0, 32, cmspi_info->dev.sectorsize);
	buf_set_u32(reg_params[2].value, 0, 32, offset);
	buf_set_u32(reg_params[3].value, 0, 32,
		(cmspi_info->mode == MODE_I2C) ? I2C_ADDR_BYTES : SPI_ADDR_BYTES);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_algorithm(target,
		0, NULL,
		ARRAY_SIZE(reg_params), reg_params,
		algorithm->address, exit_point,
		4 * count * ((cmspi_info->mode == MODE_I2C) ? 4 : 1),
		&armv7m_info);
	if (retval != ERROR_OK)
		goto err;

	result = buf_get_u32(reg_params[2].value, 0, 32);
	if (result != (offset + count)) {
		LOG_ERROR("Verify algorithm failed to compute checksum");
		retval = ERROR_FAIL;
		goto err;
	}

	image_calculate_checksum(buffer, count, &crc32);

	if (retval == ERROR_OK) {
		result = buf_get_u32(reg_params[0].value, 0, 32);
		LOG_DEBUG("addr " TARGET_ADDR_FMT ", len 0x%08" PRIx32 ", crc 0x%08" PRIx32 " 0x%08" PRIx32,
			offset + bank->base, count, ~crc32, result);
		if (~crc32 != result)
			retval = ERROR_FAIL;
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

err:
	if (cmspi_info->mode == MODE_I2C) {
		/* error recovery */
		cmspi_i2c_resync(bank);
	} else {
		/* set to input */
		set_to_output(bank, false);

		/* set NCS, ignoring errors */
		target_read_u32(target, cmspi_info->ncs.addr, &port);
		port |= (cmspi_info->ncs.mask);
		target_write_u32(target, cmspi_info->ncs.addr, port);
	}

	target_free_working_area(target, algorithm);

	return retval;
}

static int cmspi_protect(struct flash_bank *bank, int set,
	int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

/* Read a block of data from flash or write a block of data to flash */
static int cmspi_read_write_block(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count, bool write)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	struct working_area *algorithm;
	static const uint8_t *code;
	uint32_t pagesize, fifo_start, fifosize, codesize, maxsize;
	uint32_t exit_point, remaining, port;
	uint8_t cmd;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " len=0x%08" PRIx32,
		__func__, offset, count);

	/* see contrib/loaders/flash/cmspi/cmspi_read.S for src */
	static const uint8_t cmspi_read_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmspi_read.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmspi_write.S for src */
	static const uint8_t cmspi_write_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmspi_write.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmdpi_read.S for src */
	static const uint8_t cmdpi_read_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmdpi_read.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmdpi_write.S for src */
	static const uint8_t cmdpi_write_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmdpi_write.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmqpi_read.S for src */
	static const uint8_t cmqpi_read_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmqpi_read.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmqpi_write.S for src */
	static const uint8_t cmqpi_write_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmqpi_write.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmi2c_read.S for src */
	static const uint8_t cmi2c_read_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmi2c_read.inc"
	};

	/* see contrib/loaders/flash/cmspi/cmi2c_write.S for src */
	static const uint8_t cmi2c_write_code[] = {
		#include "../../../contrib/loaders/flash/cmspi/cmi2c_write.inc"
	};

	switch (cmspi_info->mode) {
		case MODE_SPI:
			code = write ? cmspi_write_code : cmspi_read_code;
			codesize = write ? sizeof(cmspi_write_code) : sizeof(cmspi_read_code);
			cmd = write ? cmspi_info->dev.pprog_cmd : cmspi_info->dev.read_cmd;
			break;

		case MODE_DPI:
			code = write ? cmdpi_write_code : cmdpi_read_code;
			codesize = write ? sizeof(cmdpi_write_code) : sizeof(cmdpi_read_code);
			cmd = write ? cmspi_info->dev.pprog_cmd : cmspi_info->dev.qread_cmd;
			break;

		case MODE_QPI:
			code = write ? cmqpi_write_code : cmqpi_read_code;
			codesize = write ? sizeof(cmqpi_write_code) : sizeof(cmqpi_read_code);
			cmd = write ? cmspi_info->dev.pprog_cmd : cmspi_info->dev.qread_cmd;
			break;

		case MODE_I2C:
			code = write ? cmi2c_write_code : cmi2c_read_code;
			codesize = write ? sizeof(cmi2c_write_code) : sizeof(cmi2c_read_code);
			cmd = cmspi_info->i2c_addr;
			break;

		default:
			return ERROR_FAIL;
	}

	/* this will overlay the words of cm*_*_code in target */
	uint32_t port_buffer[] = FILL_PORT_BUFFER(cmd);

	/* force reasonable defaults */
	fifosize = cmspi_info->dev.sectorsize ?
		cmspi_info->dev.sectorsize : cmspi_info->dev.size_in_bytes;
	pagesize = cmspi_info->dev.pagesize;
	if (pagesize == 0)
		pagesize = (fifosize <= SPIFLASH_DEF_PAGESIZE) ? fifosize : SPIFLASH_DEF_PAGESIZE;

	/* memory buffer, we assume sectorsize to be a power of 2 times pagesize */
	maxsize = target_get_working_area_avail(target);
	if (maxsize < codesize + 2 * sizeof(uint32_t) + pagesize) {
		LOG_WARNING("Not enough working area, can't do SPI %s",
			write ? "page writes" : "reads");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* fifo size at most sector size, and multiple of page size */
	maxsize -= (codesize + 2 * sizeof(uint32_t));
	fifosize = ((maxsize < fifosize) ? maxsize : fifosize) & ~(pagesize - 1);
	LOG_DEBUG("fifo_size 0x%08" PRIx32, fifosize);

	if (target_alloc_working_area_try(target,
		codesize + 2 * sizeof(uint32_t) + fifosize, &algorithm) != ERROR_OK) {
		LOG_ERROR("allocating working area failed");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* prepare read/write code, excluding port_buffer */
	retval = target_write_buffer(target, algorithm->address,
		codesize - sizeof(port_buffer), code);
	if (retval != ERROR_OK)
		goto err;

	/* prepare port_buffer values */
	retval = target_write_buffer(target, algorithm->address
		+ codesize - sizeof(port_buffer),
		sizeof(port_buffer), (uint8_t *) port_buffer);
	if (retval != ERROR_OK)
		goto err;

	/* target buffer starts right after flash_write_code, i. e.
	 * wp and rp are implicitly included in buffer!!! */
	fifo_start = algorithm->address + codesize + 2 * sizeof(uint32_t);

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* count (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* flash pagesize */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);	/* offset into flash address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* 1/2/3/4-byte address mode */
	init_reg_param(&reg_params[4], "r8", 32, PARAM_OUT);	/* fifo start */
	init_reg_param(&reg_params[5], "r9", 32, PARAM_OUT);	/* fifo end + 1 */

	buf_set_u32(reg_params[0].value, 0, 32, count);
	buf_set_u32(reg_params[1].value, 0, 32,
		write ? pagesize : cmspi_info->dev.sectorsize);
	buf_set_u32(reg_params[2].value, 0, 32, offset);
	buf_set_u32(reg_params[3].value, 0, 32,
		(cmspi_info->mode == MODE_I2C) ? I2C_ADDR_BYTES : SPI_ADDR_BYTES);
	buf_set_u32(reg_params[4].value, 0, 32, fifo_start);
	buf_set_u32(reg_params[5].value, 0, 32, fifo_start + fifosize);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* after breakpoint instruction (halfword) one nop (halfword) and
	 * port_buffer till end of code */
	exit_point = algorithm->address + codesize
		- sizeof(uint32_t) - sizeof(port_buffer);

	if (write) {
		retval = target_run_flash_async_algorithm(target, buffer, count, 1,
				0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algorithm->address + codesize,
				fifosize + 2 * sizeof(uint32_t),
				algorithm->address, exit_point,
				&armv7m_info);
	} else {
		retval = target_run_read_async_algorithm(target, buffer, count, 1,
				0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				algorithm->address + codesize,
				fifosize + 2 * sizeof(uint32_t),
				algorithm->address, exit_point,
				&armv7m_info);
	}

	remaining = buf_get_u32(reg_params[0].value, 0, 32);
	if ((retval == ERROR_OK) && remaining)
		retval = ERROR_FLASH_OPERATION_FAILED;
	if (retval != ERROR_OK) {
		offset = buf_get_u32(reg_params[2].value, 0, 32);
		LOG_ERROR("flash %s failed at address 0x%" PRIx32 ", remaining 0x%" PRIx32,
			write ? "write" : "read", offset, remaining);
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

err:
	if (cmspi_info->mode == MODE_I2C) {
		/* error recovery */
		cmspi_i2c_resync(bank);
	} else {
		/* set to input */
		set_to_output(bank, false);

		/* set NCS, ignoring errors */
		target_read_u32(target, cmspi_info->ncs.addr, &port);
		port |= (cmspi_info->ncs.mask);
		target_write_u32(target, cmspi_info->ncs.addr, port);
	}

	target_free_working_area(target, algorithm);

	return retval;
}

static int cmspi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	int sector;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > cmspi_info->dev.size_in_bytes) {
		LOG_WARNING("Write beyond end of flash. Extra data discarded.");
		count = cmspi_info->dev.size_in_bytes - offset;
	}

	/* check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset < (bank->sectors[sector].offset + bank->sectors[sector].size))
			&& ((offset + count - 1) >= bank->sectors[sector].offset)) {
			if (bank->sectors[sector].is_protected) {
				LOG_ERROR("Flash sector %d protected", sector);
				return ERROR_FLASH_PROTECTED;
			} else
				bank->sectors[sector].is_erased = -1;
		}
	}

	return cmspi_read_write_block(bank, (uint8_t *) buffer, offset, count, true);
}

static int cmspi_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
		__func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(cmspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > cmspi_info->dev.size_in_bytes) {
		LOG_WARNING("Read beyond end of flash. Extra data to be ignored.");
		count = cmspi_info->dev.size_in_bytes - offset;
	}

	return cmspi_read_write_block(bank, buffer, offset, count, false);
}

/* Find appropriate dummy setting */
static int find_sfdp_dummy(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	const int max_bytes = 64;
	uint32_t port, data;
	int len, retval;

	/* clear NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		goto err;
	CLR_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		goto err;

	/* set to output */
	retval = set_to_output(bank, true);
	if (retval != ERROR_OK)
		goto err;

	retval = cmspi_shift_out(bank, SPIFLASH_READ_SFDP);
	if (retval != ERROR_OK)
		goto err;

	/* always three address bytes, in SPI, DPI, QPI modes */
	for (len = 3; len > 0; --len) {
		retval = cmspi_shift_out(bank, 0);
		if (retval != ERROR_OK)
			goto err;
	}

	/* set to input */
	if (set_to_output(bank, false) != ERROR_OK)
		goto err;

	for (len = 0 ; len < max_bytes; len++) {
		retval = cmspi_shift_in(bank, &data);
		if (retval != ERROR_OK)
			goto err;

		if ((data & 0xFF) == 0x53) {
			LOG_DEBUG("start of SFDP header after %d dummy bytes", len);
			cmspi_info->sfdp_dummy = len;
			break;
		}
	}

	if (cmspi_info->sfdp_dummy == 0)
		LOG_DEBUG("no start of SFDP header even after %d dummy bytes", len);

err:
	/* set to input */
	set_to_output(bank, false);

	/* set NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		return retval;

	SET_PORT_BIT(port, ncs);
	return retval;
}

/* Read SFDP parameter block */
static int read_sfdp_block(struct flash_bank *bank, uint32_t addr,
	uint32_t words, uint32_t *buffer)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port, data;
	int retval, len;

	if ((target->state != TARGET_HALTED) && (target->state != TARGET_RESET)) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("%s: addr=0x%08" PRIx32 " words=0x%08" PRIx32 " dummy=%d",
		__func__, addr, words, cmspi_info->dummy);

	/* clear CLK */
	retval = target_read_u32(target, cmspi_info->clk.addr, &port);
	if (retval != ERROR_OK)
		goto err;
	CLR_PORT_BIT(port, clk);
	if (retval != ERROR_OK)
		goto err;

	/* set NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		goto err;
	SET_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		goto err;

	/* poll WIP */
	retval = wait_till_ready(bank, CMSPI_PROBE_TIMEOUT);
	if ((retval != ERROR_OK) && (retval != ERROR_FLASH_BUSY))
		return retval;

	/* if number of dummy clocks not probed yet, do it now */
	if (cmspi_info->sfdp_dummy == 0) {
		retval = find_sfdp_dummy(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	/* clear NCS */
	CLR_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		goto err;

	/* set to output */
	retval = set_to_output(bank, true);
	if (retval != ERROR_OK)
		goto err;

	retval = cmspi_shift_out(bank, SPIFLASH_READ_SFDP);
	if (retval != ERROR_OK)
		goto err;

	for (len = 3; len > 0; ) {
		retval = cmspi_shift_out(bank, (addr >> (--len << 3)) & 0xFF);
		if (retval != ERROR_OK)
			goto err;
	}

	/* set to input */
	if (set_to_output(bank, false) != ERROR_OK)
		goto err;

	retval = target_read_u32(target, cmspi_info->clk.addr, &port);
	if (retval != ERROR_OK)
		goto err;

	/* read dummy *bytes* */
	for (len = cmspi_info->sfdp_dummy ; len > 0; --len) {
		retval = cmspi_shift_in(bank, &data);
		if (retval != ERROR_OK)
			goto err;
	}

	/* read bytes */
	uint8_t *bytes = (uint8_t *) buffer;
	for (len = words << 2; len > 0; --len) {
		retval = cmspi_shift_in(bank, &data);
		if (retval != ERROR_OK)
			goto err;
		*bytes++ = data & 0xFF;
	}

	/* endian correction, sfdp data is always le uint32_t based */
	for ( ; words > 0; words--) {
		*buffer = le_to_h_u32((uint8_t *) buffer);
		buffer++;
	}

err:
	/* set to input */
	if (set_to_output(bank, false) != ERROR_OK)
		goto err;

	/* set NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		return retval;

	SET_PORT_BIT(port, ncs);
	return retval;
}

/* Return ID of flash device */
static int read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	uint32_t port, data;
	int j, k, retval, type, success;

	/* invalidate id */
	*id = 0;

	success = ERROR_FAIL;

	if ((cmspi_info->mode != MODE_SPI) &&
		(cmspi_info->mode != MODE_DPI) &&
		(cmspi_info->mode != MODE_QPI))
		return ERROR_FAIL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* clear CLK */
	retval = target_read_u32(target, cmspi_info->clk.addr, &port);
	if (retval != ERROR_OK)
		goto err;
	CLR_PORT_BIT(port, clk);
	if (retval != ERROR_OK)
		goto err;

	/* set NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		goto err;
	SET_PORT_BIT(port, ncs);
	if (retval != ERROR_OK)
		goto err;

	/* poll WIP */
	retval = wait_till_ready(bank, CMSPI_PROBE_TIMEOUT);
	if ((retval != ERROR_OK) && (retval != ERROR_FLASH_BUSY))
		return retval;

	for (type = 0; type < 2 ; type++) {
		/* clear NCS */
		retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
		if (retval != ERROR_OK)
			goto err;
		CLR_PORT_BIT(port, ncs);
		if (retval != ERROR_OK)
			goto err;

		/* set to output */
		retval = set_to_output(bank, true);
		if (retval != ERROR_OK)
			goto err;

		/* Read id: one particular flash chip switches back to SPI mode when receiving
		 * SPI_FLASH_READ_ID in QPI mode, hence try SPIFLASH_READ_MID first */
		switch (type) {
			case 0:
				success = cmspi_shift_out(bank, SPIFLASH_READ_MID);
				break;

			case 1:
				success = cmspi_shift_out(bank, SPIFLASH_READ_ID);
				break;

			default:
				return ERROR_FAIL;
		}
		if (success != ERROR_OK)
			goto err;

		/* set to input */
		if (set_to_output(bank, false) != ERROR_OK)
			goto err;

		for (k = 0; k < 3; k++) {
			for (j = 0; j < 16; j++) {
				success = cmspi_shift_in(bank, &data);
				if (success != ERROR_OK)
					goto err;
				/* discard 'continuation code' 0x7F */
				if ((data & 0xFF) != 0x7F)
					break;
				else
					data >>= 8;
			}
		}

		/* set NCS */
		retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
		if (retval != ERROR_OK)
			goto err;
		SET_PORT_BIT(port, ncs);
		if (retval != ERROR_OK)
			goto err;

		/* three bytes received, placed in bits 0 to 23, byte reversed */
		*id = ((data & 0xFF) << 16) | (data & 0xFF00) | ((data & 0xFF0000) >> 16);
		if ((*id != 0x000000) && (*id != 0xFFFFFF)) {
			/* got an apparently valid id, so done */
			break;
		}
	}

	if ((*id == 0x000000) || (*id == 0xFFFFFF)) {
		LOG_INFO("No response from flash");
		success = ERROR_FLASH_BANK_NOT_PROBED;
	}

err:
	/* set to input, ignoring errors */
	set_to_output(bank, false);

	/* set NCS */
	retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
	if (retval != ERROR_OK)
		return retval;

	SET_PORT_BIT(port, ncs);

	return success;
}

/* Get current setting of io direction registers */
static int cmspi_get_io_def(struct flash_bank *bank)
{
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t dir_reg;
	unsigned int k;
	int retval;

	k = DATA_PINS - 1;
	do {
		/* if addr or mask equals 0, ignore port direction */
		if ((cmspi_info->io[k].addr != 0) && (cmspi_info->io[k].mask != 0)) {
			if (cmspi_info->dir_inp_off == cmspi_info->dir_out_off) {
				/* read-modify-write direction register, save default direction */
				dir_reg = cmspi_info->io[k].addr + DIR_INP_OFF(k);
				retval = target_read_u32(target, dir_reg, &cmspi_info->io_def[k]);
				if (retval != ERROR_OK) {
					LOG_ERROR("dir register at 0x%08" PRIx32 " not readable", dir_reg);
					return retval;
				} else
					LOG_DEBUG("dir register %d at 0x%08" PRIx32 ": 0x%08" PRIx32,
						k, dir_reg, cmspi_info->io_def[k]);
			} else {
				/* set/clear direction register, nothing to save */
				cmspi_info->io_def[k] = 0x0;
			}
		}
	} while (k-- > 0);

	return ERROR_OK;
}

static int get_cmspi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	struct flash_device *dev = &cmspi_info->dev;

	if (!(cmspi_info->probed)) {
		snprintf(buf, buf_size,	"\ncmspi flash bank not probed yet\n");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	snprintf(buf, buf_size, "flash \'%s\', device id = 0x%06" PRIx32
		", flash size = %" PRIu32 "%sBytes\n(page size = %d, read = 0x%02" PRIx8
		", qread = 0x%02" PRIx8 ", pprog = 0x%02" PRIx8
		", mass_erase = 0x%02" PRIx8 ", sector_size = %" PRIu32 "%sBytes"
		", sector_erase = 0x%02" PRIx8 ")",
		dev->name, dev->device_id,
		(dev->size_in_bytes / 4096) ? dev->size_in_bytes / 1024 : dev->size_in_bytes,
		(dev->size_in_bytes / 4096) ? "k" : "", dev->pagesize, dev->read_cmd,
		dev->qread_cmd, dev->pprog_cmd, dev->chip_erase_cmd,
		(dev->sectorsize / 4096) ? dev->sectorsize / 1024 : dev->sectorsize,
		(dev->sectorsize / 4096) ? "k" : "", dev->erase_cmd);

	return ERROR_OK;
}

/* Read id from flash chip, identify flash  */
static int cmspi_probe(struct flash_bank *bank)
{
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;
	struct flash_sector *sectors;
	const struct flash_device *p;
	uint32_t id = 0;
	int sector, retval;

	if (cmspi_info->probed) {
		bank->size = 0;
		bank->num_sectors = 0;
		if (bank->sectors)
			free(bank->sectors);
		bank->sectors = NULL;
		cmspi_info->probed = false;
		cmspi_info->sfdp_dummy = 0;
	}
	memset(&cmspi_info->dev, 0, sizeof(cmspi_info->dev));

	retval = cmspi_get_io_def(bank);
	if (retval != ERROR_OK)
		return retval;

	switch (cmspi_info->mode) {
		case MODE_SPI:
			/* fall through */

		case MODE_DPI:
			/* in SPI/DPI modes set IO3/NHOLD and IO2/NWP to output high */
			retval = deactivate_nhold_nwp(bank);
			if (retval != ERROR_OK)
				return retval;
			/* fall through */

		case MODE_QPI:
			/* read and decode flash ID */
			retval = read_flash_id(bank, &id);
			LOG_DEBUG("id 0x%06" PRIx32, id);

			if (retval == ERROR_FLASH_BANK_NOT_PROBED) {
				/* no id retrieved, so id must be set manually */
				LOG_INFO("no id - set flash parameters manually");
				return ERROR_OK;
			}
			if (retval != ERROR_OK)
				return retval;

			/* identify flash */
			cmspi_info->dev.name = NULL;
			for (p = flash_devices; id && p->name ; p++) {
				if (p->device_id == id) {
					memcpy(&cmspi_info->dev, p, sizeof(cmspi_info->dev));
					if (p->size_in_bytes / 4096)
						LOG_INFO("flash \'%s\' id = 0x%06" PRIx32 ", size = %" PRIu32
							"kbytes", p->name, id, p->size_in_bytes / 1024);
					else
						LOG_INFO("flash \'%s\' id = 0x%06" PRIx32 ", size = %" PRIu32
							"bytes", p->name, id, p->size_in_bytes);
					break;
				}
			}

			if (id && !p->name) {
				/* chip not been identified by id, then try SFDP */
				retval = spi_sfdp(bank, &cmspi_info->dev, &read_sfdp_block);

				/* save retrieved *good* id as spi_sfdp clears all info */
				cmspi_info->dev.device_id = id;

				if (retval != ERROR_OK) {
					/* even not identified by SFDP, then give up */
					LOG_INFO("Unknown flash device id = 0x%06" PRIx32
						" - set flash parameters manually", id);
					return ERROR_OK;
				}
			}

			/* set correct size value */
			bank->size = cmspi_info->dev.size_in_bytes;

			/* create and fill sectors array, 'no sectors' interpreted as 1 sector */
			if (cmspi_info->dev.sectorsize == 0)
				cmspi_info->dev.sectorsize = cmspi_info->dev.size_in_bytes;
			/* if no pagesize, then use reasonable default instead */
			if (cmspi_info->dev.pagesize == 0)
				cmspi_info->dev.pagesize = SPIFLASH_DEF_PAGESIZE;

			bank->num_sectors = cmspi_info->dev.size_in_bytes / cmspi_info->dev.sectorsize;
			sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
			if (sectors == NULL) {
				LOG_ERROR("Not enough memory");
				return ERROR_FAIL;
			}

			for (sector = 0; sector < bank->num_sectors; sector++) {
				sectors[sector].offset = sector * (cmspi_info->dev.sectorsize);
				sectors[sector].size = cmspi_info->dev.sectorsize;
				sectors[sector].is_erased = -1;
				sectors[sector].is_protected = 0;
			}

			bank->sectors = sectors;
			cmspi_info->probed = true;
			break;

		case MODE_I2C:
			LOG_DEBUG("Auto-probe not available in I2C mode - set parameters manually");
			return ERROR_OK;

		default:
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int cmspi_auto_probe(struct flash_bank *bank)
{
	struct cmspi_flash_bank *cmspi_info = bank->driver_priv;

	if (cmspi_info->probed)
		return ERROR_OK;
	return cmspi_probe(bank);
}

static int cmspi_protect_check(struct flash_bank *bank)
{
	/* nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

COMMAND_HANDLER(cmspi_handle_mass_erase)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct cmspi_flash_bank *cmspi_info;
	struct duration bench;
	uint32_t port, data;
	int retval, sector;
	uint8_t *buffer = NULL;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	cmspi_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(cmspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (sector = 0; sector < bank->num_sectors; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FLASH_PROTECTED;
		}
	}

	duration_start(&bench);

	if (cmspi_info->dev.chip_erase_cmd != 0x00) {
		retval = cmspi_write_enable(bank);
		if (retval != ERROR_OK)
			return retval;

		/* set to output */
		retval = set_to_output(bank, true);
		if (retval != ERROR_OK)
			goto err;

		/* Send Mass Erase command */
		/* clear NCS */
		retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
		if (retval != ERROR_OK)
			goto err;
		CLR_PORT_BIT(port, ncs);
		if (retval != ERROR_OK)
			goto err;

		/* send command byte */
		retval = cmspi_shift_out(bank, cmspi_info->dev.chip_erase_cmd);
		if (retval != ERROR_OK)
			goto err;

err:
		/* set to input, ignoring errors */
		set_to_output(bank, false);

		/* set NCS */
		retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
		if (retval != ERROR_OK)
			return retval;

		SET_PORT_BIT(port, ncs);
		if (retval != ERROR_OK)
			return retval;

		/* read flash status register */
		retval = read_status_reg(bank, &data);
		if (retval != ERROR_OK)
			return retval;

		/* check for command in progress for flash */
		if ((data & SPIFLASH_WE_BIT) == 0) {
			LOG_ERROR("Mass erase command not accepted by flash. Status=0x%08" PRIx32, data);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		/* poll WIP for end of self timed Sector Erase cycle */
		retval = wait_till_ready(bank, CMSPI_MASS_ERASE_TIMEOUT);
	} else {
		uint32_t count, offset;

		LOG_INFO("Mass erase not available - emulating via writing 0xFF");
		count = cmspi_info->dev.size_in_bytes;
		count = (count < (1UL << 20)) ? count : (1UL << 20);
		while (!(buffer = malloc(count))) {
			count >>= 1;
			if (count < (1UL << 12)) {
				LOG_ERROR("Not enough memory");
				return ERROR_FAIL;
			}
		}
		memset(buffer, 0xFF, count);
		for (offset = 0; offset < cmspi_info->dev.size_in_bytes; offset += count) {
			retval = cmspi_read_write_block(bank, buffer, offset, count, true);
			if (retval != ERROR_OK)
				break;
		}
		free(buffer);
	}

	duration_measure(&bench);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (sector = 0; sector < bank->num_sectors; sector++)
			bank->sectors[sector].is_erased = 1;

		command_print(CMD, "cmspi mass erase completed in"
			" %fs (%0.3f KiB/s)", duration_elapsed(&bench),
			duration_kbps(&bench, bank->size));
	} else {
		command_print(CMD, "cmspi mass erase failed after %fs",
			duration_elapsed(&bench));
	}

	return retval;
}

COMMAND_HANDLER(cmspi_handle_set)
{
	struct flash_bank *bank = NULL;
	struct cmspi_flash_bank *cmspi_info = NULL;
	struct flash_sector *sectors = NULL;
	uint32_t temp;
	unsigned int index = 1;
	int retval, i2c_ack;

	LOG_DEBUG("%s", __func__);

	/* there are 3 mandatory arguments: devname, size_in_bytes, pagesize */
	if (index + 3 > CMD_ARGC) {
		command_print(CMD, "cmspi: not enough arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;
	cmspi_info = bank->driver_priv;

	/* invalidate all old info */
	if (cmspi_info->probed) {
		bank->size = 0;
		bank->num_sectors = 0;
		if (bank->sectors)
			free(bank->sectors);
		bank->sectors = NULL;
		cmspi_info->probed = false;
		cmspi_info->sfdp_dummy = 0;
	}
	memset(&cmspi_info->dev, 0, sizeof(cmspi_info->dev));

	retval = cmspi_get_io_def(bank);
	if (retval != ERROR_OK)
		return retval;

	strncpy(cmspi_info->devname, CMD_ARGV[index++], sizeof(cmspi_info->devname) - 1);
	cmspi_info->devname[sizeof(cmspi_info->devname) - 1] = '\0';

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
	cmspi_info->dev.size_in_bytes = temp;
	if (log2u(cmspi_info->dev.size_in_bytes) < 8) {
		command_print(CMD, "cmspi: device size must be 2^n with n >= 8");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], cmspi_info->dev.pagesize);
	if (cmspi_info->dev.pagesize == 0)
		cmspi_info->dev.pagesize = SPIFLASH_DEF_PAGESIZE;
	if ((cmspi_info->dev.pagesize > cmspi_info->dev.size_in_bytes) ||
		(log2u(cmspi_info->dev.pagesize) < 0)) {
		command_print(CMD, "cmspi: page size must be 2^n and <= device size");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	switch (cmspi_info->mode) {
		case MODE_SPI:
			/* fall through */

		case MODE_DPI:
			/* fall through */

		case MODE_QPI:
			if (index + 3 > CMD_ARGC) {
				command_print(CMD, "cmspi: not enough arguments");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmspi_info->dev.read_cmd);
			if ((cmspi_info->dev.read_cmd != 0x03) &&
				(cmspi_info->dev.read_cmd != 0x13)) {
				command_print(CMD, "cmspi: only 0x03/0x13 READ allowed");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmspi_info->dev.qread_cmd);
			if ((cmspi_info->dev.qread_cmd != 0x00) &&
				(cmspi_info->dev.qread_cmd != 0x0B) &&
				(cmspi_info->dev.qread_cmd != 0x0C) &&
				(cmspi_info->dev.qread_cmd != 0x3B) &&
				(cmspi_info->dev.qread_cmd != 0x3C) &&
				(cmspi_info->dev.qread_cmd != 0x6B) &&
				(cmspi_info->dev.qread_cmd != 0x6C) &&
				(cmspi_info->dev.qread_cmd != 0xBB) &&
				(cmspi_info->dev.qread_cmd != 0xBC) &&
				(cmspi_info->dev.qread_cmd != 0xEB) &&
				(cmspi_info->dev.qread_cmd != 0xEC)) {
				command_print(CMD, "cmspi: only 0x0B/0x0C/0x3B/0x3C/"
					"0x6B/0x6C/0xBB/0xBC/0xEB/0xEC QREAD allowed");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmspi_info->dev.pprog_cmd);
			if ((cmspi_info->dev.pprog_cmd != 0x02) &&
				(cmspi_info->dev.pprog_cmd != 0x12) &&
				(cmspi_info->dev.pprog_cmd != 0x32)) {
				command_print(CMD, "cmspi: only 0x02/0x12/0x32 PPRG allowed");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			if (index < CMD_ARGC)
				COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmspi_info->dev.chip_erase_cmd);
			else
				cmspi_info->dev.chip_erase_cmd = 0x00;

			if (index < CMD_ARGC) {
				COMMAND_PARSE_NUMBER(u32, CMD_ARGV[index++], temp);
				cmspi_info->dev.sectorsize = temp;
				if ((cmspi_info->dev.sectorsize > cmspi_info->dev.size_in_bytes) ||
					(cmspi_info->dev.sectorsize < cmspi_info->dev.pagesize) ||
					(log2u(cmspi_info->dev.sectorsize) < 0)) {
					command_print(CMD, "cmspi: sector size must be 2^n and <= device size");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

				if (index < CMD_ARGC)
					COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmspi_info->dev.erase_cmd);
				else {
					command_print(CMD, "cmspi: erase command missing");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			} else {
				/* no sector size / sector erase cmd given, treat whole bank as a single sector */
				cmspi_info->dev.erase_cmd = 0x00;
				cmspi_info->dev.sectorsize = cmspi_info->dev.size_in_bytes;
			}
			break;

		case MODE_I2C:
			if (index + 3  > CMD_ARGC) {
				command_print(CMD, "cmspi: missing I2C addr, bits or delay ");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmspi_info->i2c_addr);
			if ((cmspi_info->i2c_addr & 0xF0) != I2C_ADDR_EEPROM_NORM)
				LOG_INFO("unusual I2C address - please check");
			if (cmspi_info->i2c_addr & 0x1) {
				LOG_ERROR("R/W bit must be cleared");
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmspi_info->i2c_bits);
			if (cmspi_info->i2c_bits > 3) {
				LOG_ERROR("at most 3 memory address bits in I2C address");
				return ERROR_FAIL;
			}
			if (cmspi_info->i2c_bits != 0) {
				/* consistency check: if any memory address bit in I2C address */
				if ((log2u(cmspi_info->dev.size_in_bytes) - cmspi_info->i2c_bits) & 0x7) {
					/* then following address bytes must not have any don't care bits */
					LOG_ERROR("no don't care address bits in memory address byte(s) allowed");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
				if (cmspi_info->i2c_addr & ((1UL << cmspi_info->i2c_bits) - 1) << 1) {
					/* and the memory address bits in I2C address byte must cleared */
					LOG_ERROR("memory address bits in I2C address must be 0");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}

			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmspi_info->dummy);

			/* just in case ... */
			retval = cmspi_i2c_resync(bank);
			if (retval != ERROR_OK) {
				LOG_ERROR("resyncing i2c failed");
				return ERROR_FAIL;
			}

			/* check that memory is present */
			retval = cmspi_i2c_start_stop(bank, true);
			if (retval != ERROR_OK) {
				LOG_ERROR("generating i2c start failed");
				return ERROR_FAIL;
			}
			i2c_ack = cmspi_shift_out(bank, cmspi_info->i2c_addr | 0x0);
			cmspi_i2c_start_stop(bank, false);
			if (retval != ERROR_OK) {
				LOG_ERROR("generating i2c stop failed");
				return ERROR_FAIL;
			}
			if (i2c_ack != ERROR_OK) {
				if (i2c_ack == ERROR_FLASH_OPERATION_FAILED)
					LOG_ERROR("i2c no ack from memory");
				else
					LOG_ERROR("i2c transfer error");
				return retval;
			}
			LOG_INFO("i2c device at 0x%02" PRIx8 " detected", cmspi_info->i2c_addr);

			/* treat whole bank as a single sector */
			cmspi_info->dev.erase_cmd = 0x00;
			cmspi_info->dev.sectorsize = cmspi_info->dev.size_in_bytes;
			break;

		default:
			return ERROR_FAIL;
	}

	if (index < CMD_ARGC) {
		command_print(CMD, "cmspi: extra arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* set correct size value */
	bank->size = cmspi_info->dev.size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors =
		cmspi_info->dev.size_in_bytes / cmspi_info->dev.sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("Not enough memory");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * (cmspi_info->dev.sectorsize);
		sectors[sector].size = cmspi_info->dev.sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	cmspi_info->dev.name = cmspi_info->devname;
	if (cmspi_info->dev.size_in_bytes / 1024)
		LOG_INFO("flash \'%s\' id = unknown\nflash size = %" PRIu32 "kbytes",
			cmspi_info->dev.name, cmspi_info->dev.size_in_bytes / 1024);
	else
		LOG_INFO("flash \'%s\' id = unknown\nflash size = %" PRIu32 "bytes",
			cmspi_info->dev.name, cmspi_info->dev.size_in_bytes);

	/* in SPI/DPI modes set IO3/NHOLD and IO2/NWP to output high */
	retval = deactivate_nhold_nwp(bank);
	if (retval != ERROR_OK)
		return retval;

	cmspi_info->probed = true;

	return ERROR_OK;
}

COMMAND_HANDLER(cmspi_handle_cmd)
{
	struct target *target = NULL;
	struct flash_bank *bank;
	struct cmspi_flash_bank *cmspi_info;
	uint32_t port, data;
	uint16_t num_bytes;
	uint8_t addr_byte, cmd_byte;
	unsigned int index = 1;
	char temp[6], output[(2 + 2 * (1 << CHAR_BIT)) * 3 + 10];
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC > (1 << CHAR_BIT) + 3)	{
		LOG_ERROR("at most %d bytes may be send", (1 << CHAR_BIT));
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	cmspi_info = bank->driver_priv;

	/* at most 256 bytes may be read */
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[index++], num_bytes);
	if (num_bytes > (1 << CHAR_BIT)) {
		LOG_ERROR("at most %d bytes may be send", (1 << CHAR_BIT));
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	switch (cmspi_info->mode) {
		case MODE_SPI:
			/* fall through */

		case MODE_DPI:
			/* in SPI/DPI modes set IO3/NHOLD and IO2/NWP to output high */
			retval = deactivate_nhold_nwp(bank);
			if (retval != ERROR_OK)
				return retval;
			/* fall through */

		case MODE_QPI:
			if (index >= CMD_ARGC)
				return ERROR_COMMAND_SYNTAX_ERROR;
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], cmd_byte);

			/* clear NCS */
			retval = target_read_u32(target, cmspi_info->ncs.addr, &port);
			if (retval != ERROR_OK)
				goto err;
			CLR_PORT_BIT(port, ncs);
			if (retval != ERROR_OK)
				goto err;

			/* set to output */
			retval = set_to_output(bank, true);
			if (retval != ERROR_OK)
				goto err;

			/* send command byte */
			snprintf(output, sizeof(output), "%s: %02" PRIx8 " ",
				(cmspi_info->mode == MODE_SPI) ? "spi" :
					(cmspi_info->mode == MODE_DPI) ? "dpi" : "qpi", cmd_byte & 0xFF);
			retval = cmspi_shift_out(bank, cmd_byte);
			if (retval != ERROR_OK)
				goto err;

			/* send additional bytes */
			for ( ; index < CMD_ARGC; index++) {
				COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index], cmd_byte);
				snprintf(temp, sizeof(temp), "%02" PRIx8 " ", cmd_byte & 0xFF);
				retval = cmspi_shift_out(bank, cmd_byte);
				if (retval != ERROR_OK)
					goto err;
				strncat(output, temp, sizeof(output) - strlen(output) - 1);
			}

			/* set to input */
			retval = set_to_output(bank, false);
			if (retval != ERROR_OK)
				goto err;

			strncat(output, "-> ", sizeof(output) - strlen(output) - 1);

			/* now read the response bytes */
			for ( ; num_bytes > 0; num_bytes--)	{
				retval = cmspi_shift_in(bank, &data);
				if (retval != ERROR_OK)
					goto err;
				snprintf(temp, sizeof(temp), "%02" PRIx8 " ", data & 0xFF);
				strncat(output, temp, sizeof(output) - strlen(output) - 1);
			}
			command_print(CMD, "%s", output);
			break;

		case MODE_I2C:
			cmspi_i2c_resync(bank);

			/* unconditionally generate start condition */
			retval = cmspi_i2c_start_stop(bank, true);
			if (retval != ERROR_OK)
				goto err;

			snprintf(output, sizeof(output), "i2c:");
			if (index < CMD_ARGC) {
				COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index++], addr_byte);
				if (index < CMD_ARGC) {
					/* there are bytes to write to slave */
					addr_byte &= 0xFE;
					retval = cmspi_shift_out(bank, addr_byte);
					snprintf(temp, sizeof(temp), " (%02" PRIx8 ")", addr_byte);
					if (retval != ERROR_OK) {
						if (retval == ERROR_FLASH_OPERATION_FAILED)
							LOG_ERROR("i2c no ack on write to addr %02" PRIx8, addr_byte);
						goto err;
					}
					strncat(output, temp, sizeof(output) - strlen(output) - 1);
					/* write bytes to slave */
					for ( ; index < CMD_ARGC; index++) {
						COMMAND_PARSE_NUMBER(u8, CMD_ARGV[index], cmd_byte);
						snprintf(temp, sizeof(temp), " %02" PRIx8, cmd_byte);
						retval = cmspi_shift_out(bank, cmd_byte);
						if (retval != ERROR_OK) {
							if (retval == ERROR_FLASH_OPERATION_FAILED)
								LOG_ERROR("i2c no ack on write");
							goto err;
						}
						strncat(output, temp, sizeof(output) - strlen(output) - 1);
					}
					if (num_bytes > 0) {
						/* send REPEATED start condition */
						retval = cmspi_i2c_start_stop(bank, true);
						if (retval != ERROR_OK)
							goto err;
					}
				}

				strncat(output, " ->", sizeof(output) - strlen(output) - 1);
				if (num_bytes > 0) {
					/* now read the response bytes */
					addr_byte |= 0x01;
					retval = cmspi_shift_out(bank, addr_byte);
					snprintf(temp, sizeof(temp), " (%02" PRIx8 ")", addr_byte);
					if (retval != ERROR_OK) {
						if (retval == ERROR_FLASH_OPERATION_FAILED)
							LOG_ERROR("i2c no ack on read from addr %02" PRIx8, addr_byte);
						goto err;
					}
					strncat(output, temp, sizeof(output) - strlen(output) - 1);
					for ( ; num_bytes > 0; num_bytes--) {
						/* generate ACK for each but the last byte */
						data = (num_bytes == 1) ? 0 : 1;
						retval = cmspi_shift_in(bank, &data);
						if (retval != ERROR_OK)
							goto err;
						snprintf(temp, sizeof(temp), " %02" PRIx8, data & 0xFF);
						strncat(output, temp, sizeof(output) - strlen(output) - 1);
					}
				}
			}

			/* unconditionally generate stop condition */
			retval = cmspi_i2c_start_stop(bank, false);
			if (retval != ERROR_OK)
				goto err;
			command_print(CMD, "%s", output);
			break;

		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

err:
	if (cmspi_info->mode == MODE_I2C) {
		/* error recovery */
		cmspi_i2c_resync(bank);
	} else {
		/* set to input */
		set_to_output(bank, false);

		/* set NCS, ignoring errors */
		target_read_u32(target, cmspi_info->ncs.addr, &port);
		port |= (cmspi_info->ncs.mask);
		target_write_u32(target, cmspi_info->ncs.addr, port);
	}

	return retval;
}

COMMAND_HANDLER(cmspi_handle_spi)
{
	struct flash_bank *bank;
	struct cmspi_flash_bank *cmspi_info;
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	cmspi_info = bank->driver_priv;
	if ((cmspi_info->mode != MODE_SPI) &&
		(cmspi_info->mode != MODE_DPI) &&
		(cmspi_info->mode != MODE_QPI)) {
		LOG_ERROR("not available in this mode");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	cmspi_info->mode = MODE_SPI;

	/* force re-probing of SFDP dummy bytes */
	cmspi_info->sfdp_dummy = 0;

	/* set IO3/NHOLD and IO2/NWP to high */
	deactivate_nhold_nwp(bank);

	return ERROR_OK;
}

COMMAND_HANDLER(cmspi_handle_dpi)
{
	struct flash_bank *bank;
	struct cmspi_flash_bank *cmspi_info;
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	cmspi_info = bank->driver_priv;
	if ((cmspi_info->mode != MODE_SPI) &&
		(cmspi_info->mode != MODE_DPI) &&
		(cmspi_info->mode != MODE_QPI)) {
		LOG_ERROR("not available in this mode");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (cmspi_info->dev.qread_cmd == 0)
		LOG_WARNING("DPI not available for this device");

	/* number of dummy clocks */
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], cmspi_info->dummy);

	cmspi_info->mode = MODE_DPI;

	/* force re-probing of SFDP dummy bytes */
	cmspi_info->sfdp_dummy = 0;

	/* set IO3/NHOLD and IO2/NWP to high */
	deactivate_nhold_nwp(bank);

	return ERROR_OK;
}

COMMAND_HANDLER(cmspi_handle_qpi)
{
	struct flash_bank *bank;
	struct cmspi_flash_bank *cmspi_info;
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	cmspi_info = bank->driver_priv;
	if ((cmspi_info->mode != MODE_SPI) &&
		(cmspi_info->mode != MODE_DPI) &&
		(cmspi_info->mode != MODE_QPI)) {
		LOG_ERROR("not available in this mode");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if ((cmspi_info->io[3].addr == 0) ||
		(cmspi_info->io[2].addr == 0)) {
		command_print(CMD, "cmspi: no IO2/IO3, QPI mode not possible");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (cmspi_info->dev.qread_cmd == 0)
		LOG_WARNING("QPI not available for this device");

	/* number of dummy clocks */
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], cmspi_info->dummy);

	cmspi_info->mode = MODE_QPI;

	/* force re-probing of SFDP dummy bytes */
	cmspi_info->sfdp_dummy = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(cmspi_handle_calib)
{
	struct flash_bank *bank;
	struct cmspi_flash_bank *cmspi_info;
	double time1 = 0, time2 = 0;
	uint8_t dummy, delay;
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	cmspi_info = bank->driver_priv;
	if (cmspi_info->mode != MODE_I2C) {
		LOG_ERROR("not available in this mode");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* save previously set value */
	dummy = cmspi_info->dummy;

	/* use this value for measurement */
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], delay);
	cmspi_info->dummy = delay;

	cmspi_i2c_calib(bank, 1000, &time1);
	cmspi_i2c_calib(bank, 10000, &time2);

	/* restore old value */
	cmspi_info->dummy = dummy;

	LOG_INFO("i2c_calib: %.1f us per SCL cycle", (time2 - time1) * 1.0E+6 / 9000);

	return ERROR_OK;
}

COMMAND_HANDLER(cmspi_handle_scan)
{
	struct flash_bank *bank;
	struct cmspi_flash_bank *cmspi_info;
	int retval;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	cmspi_info = bank->driver_priv;
	if (cmspi_info->mode != MODE_I2C) {
		LOG_ERROR("not available in this mode");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	cmspi_i2c_scan(bank);

	return ERROR_OK;
}

static const struct command_registration cmspi_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = cmspi_handle_mass_erase,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Mass erase entire flash device.",
	},
	{
		.name = "set",
		.handler = cmspi_handle_set,
		.mode = COMMAND_EXEC,
		.usage = "( bank_id name chip_size page_size read_cmd qread_cmd pprg_cmd "
			"[ mass_erase_cmd ] [ sector_size sector_erase_cmd ] "
			"| bank_id name chip_size page_size i2c_addr i2c_bits delay )",
		.help = "Set device parameters if not autodetected.",
	},
	{
		.name = "cmd",
		.handler = cmspi_handle_cmd,
		.mode = COMMAND_EXEC,
		.usage = "bank_id num_resp cmd_byte ...",
		.help = "Send low-level command cmd_byte and following bytes, read num_bytes.",
	},
	{
		.name = "spi",
		.handler = cmspi_handle_spi,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Switches driver to SPI mode.",
	},
	{
		.name = "dpi",
		.handler = cmspi_handle_dpi,
		.mode = COMMAND_EXEC,
		.usage = "bank_id dummy_clocks",
		.help = "Switches driver to DPI mode.",
	},
	{
		.name = "qpi",
		.handler = cmspi_handle_qpi,
		.mode = COMMAND_EXEC,
		.usage = "bank_id dummy_clocks",
		.help = "Switches driver to QPI mode.",
	},
	{
		.name = "calib",
		.handler = cmspi_handle_calib,
		.mode = COMMAND_EXEC,
		.usage = "bank_id delay",
		.help = "Measures SCL cycle time using delay",
	},
	{
		.name = "scan",
		.handler = cmspi_handle_scan,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Scans all 7-bit and 10-bit I2C addresses",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cmspi_command_handlers[] = {
	{
		.name = "cmspi",
		.mode = COMMAND_ANY,
		.help = "cmspi flash command group",
		.usage = "",
		.chain = cmspi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver cmspi_flash = {
	.name = "cmspi",
	.commands = cmspi_command_handlers,
	.flash_bank_command = cmspi_flash_bank_command,
	.erase = cmspi_erase,
	.protect = cmspi_protect,
	.write = cmspi_write,
	.read = cmspi_read,
	.verify = cmspi_verify,
	.probe = cmspi_probe,
	.auto_probe = cmspi_auto_probe,
	.erase_check = cmspi_blank_check,
	.protect_check = cmspi_protect_check,
	.info = get_cmspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
	.usage = "$_FLASHNAME cmspi base 0 0 0 $_TARGETNAME inp_off "
		"ncs_addr ncs_bit clk_addr clk_bit "
		"io3_addr io3_bit io3_dir_inp_off io3_dir_out_off io3_exor "
		"io2_addr io2_bit io2_dir_inp_off io2_dir_out_off io2_exor "
		"io1_addr io1_bit io1_dir_inp_off io1_dir_out_off io1_exor "
		"io0_addr io0_bit io0_dir_inp_off io0_dir_out_off io0_exor",
};
