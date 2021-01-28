/***************************************************************************
 *   Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>       *
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

#include "target.h"
#include "jtag/jtag.h"
#include "avr32_jtag.h"
#include "avr32_mem.h"

int avr32_jtag_read_memory32(struct avr32_jtag *jtag_info,
	uint32_t addr, int count, uint32_t *buffer)
{
	int i, retval;
	uint32_t data;

	for (i = 0; i < count; i++) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i*4, &data);

		if (retval != ERROR_OK)
			return retval;

		/* XXX: Assume AVR32 is BE */
		buffer[i] = be_to_h_u32((uint8_t *)&data);
	}

	return ERROR_OK;
}

int avr32_jtag_read_memory16(struct avr32_jtag *jtag_info,
	uint32_t addr, int count, uint16_t *buffer)
{
	int i, retval;
	uint32_t data;

	i = 0;

	/* any unaligned half-words? */
	if (addr & 3) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i*2, &data);

		if (retval != ERROR_OK)
			return retval;

		/* XXX: Assume AVR32 is BE */
		data = be_to_h_u32((uint8_t *)&data);
		buffer[i] = (data >> 16) & 0xffff;
		i++;
	}

	/* read all complete words */
	for (; i < (count & ~1); i += 2) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i*2, &data);

		if (retval != ERROR_OK)
			return retval;

		/* XXX: Assume AVR32 is BE */
		data = be_to_h_u32((uint8_t *)&data);
		buffer[i] = data & 0xffff;
		buffer[i+1] = (data >> 16) & 0xffff;
	}

	/* last halfword */
	if (i < count) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i*2, &data);

		if (retval != ERROR_OK)
			return retval;

		/* XXX: Assume AVR32 is BE */
		data = be_to_h_u32((uint8_t *)&data);
		buffer[i] = data & 0xffff;
	}

	return ERROR_OK;
}

int avr32_jtag_read_memory8(struct avr32_jtag *jtag_info,
	uint32_t addr, int count, uint8_t *buffer)
{
	int i, j, retval;
	uint8_t data[4];
	i = 0;

	/* Do we have non-aligned bytes? */
	if (addr & 3) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i, (uint32_t *)(void *)data);

		if (retval != ERROR_OK)
			return retval;

		for (j = addr & 3; (j < 4) && (i < count); j++, i++)
			buffer[i] = data[3-j];
	}

	/* read all complete words */
	for (; i < (count & ~3); i += 4) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i, (uint32_t *)(void *)data);

		if (retval != ERROR_OK)
			return retval;

		for (j = 0; j < 4; j++)
			buffer[i+j] = data[3-j];
	}

	/* remaining bytes */
	if (i < count) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i, (uint32_t *)(void *)data);

		if (retval != ERROR_OK)
			return retval;

		for (j = 0; i + j < count; j++)
			buffer[i+j] = data[3-j];
	}

	return ERROR_OK;
}

int avr32_jtag_write_memory32(struct avr32_jtag *jtag_info,
	uint32_t addr, int count, const uint32_t *buffer)
{
	int i, retval;
	uint32_t data;

	for (i = 0; i < count; i++) {
		/* XXX: Assume AVR32 is BE */
		h_u32_to_be((uint8_t *)&data, buffer[i]);
		retval = avr32_jtag_mwa_write(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i*4, data);

		if (retval != ERROR_OK)
			return retval;

	}

	return ERROR_OK;
}

int avr32_jtag_write_memory16(struct avr32_jtag *jtag_info,
	uint32_t addr, int count, const uint16_t *buffer)
{
	int i, retval;
	uint32_t data;
	uint32_t data_out;

	i = 0;

	/*
	 * Do we have any non-aligned half-words?
	 */
	if (addr & 3) {
		/*
		 * mwa_read will read whole world, no need to fiddle
		 * with address. It will be truncated in set_addr
		 */
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr, &data);

		if (retval != ERROR_OK)
			return retval;

		data = be_to_h_u32((uint8_t *)&data);
		data = (buffer[i] << 16) | (data & 0xffff);
		h_u32_to_be((uint8_t *)&data_out, data);

		retval = avr32_jtag_mwa_write(jtag_info, SLAVE_HSB_UNCACHED,
				addr, data_out);

		if (retval != ERROR_OK)
			return retval;

		i++;
	}

	/* write all complete words */
	for (; i < (count & ~1); i += 2) {
		/* XXX: Assume AVR32 is BE */
		data = (buffer[i+1] << 16) | buffer[i];
		h_u32_to_be((uint8_t *)&data_out, data);

		retval = avr32_jtag_mwa_write(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i*2, data_out);

		if (retval != ERROR_OK)
			return retval;
	}

	/* last halfword */
	if (i < count) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i*2, &data);

		if (retval != ERROR_OK)
			return retval;

		data = be_to_h_u32((uint8_t *)&data);
		data &= ~0xffff;
		data |= buffer[i];
		h_u32_to_be((uint8_t *)&data_out, data);

		retval = avr32_jtag_mwa_write(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i*2, data_out);

		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int avr32_jtag_write_memory8(struct avr32_jtag *jtag_info,
	uint32_t addr, int count, const uint8_t *buffer)
{
	int i, j, retval;
	uint32_t data;
	uint32_t data_out;

	i = 0;

	/*
	 * Do we have any non-aligned bytes?
	 */
	if (addr & 3) {
		/*
		 * mwa_read will read whole world, no need to fiddle
		 * with address. It will be truncated in set_addr
		 */
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr, &data);

		if (retval != ERROR_OK)
			return retval;

		data = be_to_h_u32((uint8_t *)&data);
		for (j = addr & 3; (j < 4) && (i < count); j++, i++) {
			data &= ~(0xff << j*8);
			data |= (buffer[i] << j*8);
		}

		h_u32_to_be((uint8_t *)&data_out, data);
		retval = avr32_jtag_mwa_write(jtag_info, SLAVE_HSB_UNCACHED,
				addr, data_out);

		if (retval != ERROR_OK)
			return retval;
	}


	/* write all complete words */
	for (; i < (count & ~3); i += 4) {
		data = 0;

		for (j = 0; j < 4; j++)
			data |= (buffer[j+i] << j*8);

		h_u32_to_be((uint8_t *)&data_out, data);

		retval = avr32_jtag_mwa_write(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i, data_out);

		if (retval != ERROR_OK)
			return retval;
	}

	/*
	 * Write trailing bytes
	 */
	if (i < count) {
		retval = avr32_jtag_mwa_read(jtag_info, SLAVE_HSB_UNCACHED,
				addr + i, &data);

		if (retval != ERROR_OK)
			return retval;

		data = be_to_h_u32((uint8_t *)&data);
		for (j = 0; i < count; j++, i++) {
			data &= ~(0xff << j*8);
			data |= (buffer[j+i] << j*8);
		}

		h_u32_to_be((uint8_t *)&data_out, data);

		retval = avr32_jtag_mwa_write(jtag_info, SLAVE_HSB_UNCACHED,
				addr+i, data_out);

		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}
