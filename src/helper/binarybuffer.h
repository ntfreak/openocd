/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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

#ifndef OPENOCD_HELPER_BINARYBUFFER_H
#define OPENOCD_HELPER_BINARYBUFFER_H

#include "list.h"

/** @file
 * Support functions to access arbitrary bits in a byte array
 */

/**
 * Sets @c num bits in @c _buffer, starting at the @c first bit,
 * using the bits in @c value.  This routine fast-paths writes
 * of little-endian, byte-aligned, 32-bit words.
 * @param _buffer The buffer whose bits will be set.
 *	Do not use uninitialized buffer or clang static analyzer emits a warning.
 * @param first The bit offset in @c _buffer to start writing (0-31).
 * @param num The number of bits from @c value to copy (1-32).
 * @param value Up to 32 bits that will be copied to _buffer.
 */
static inline void buf_set_u32(uint8_t *_buffer,
	unsigned first, unsigned num, uint32_t value)
{
	uint8_t *buffer = _buffer;

	if ((num == 32) && (first == 0)) {
		buffer[3] = (value >> 24) & 0xff;
		buffer[2] = (value >> 16) & 0xff;
		buffer[1] = (value >> 8) & 0xff;
		buffer[0] = (value >> 0) & 0xff;
	} else {
		for (unsigned i = first; i < first + num; i++) {
			if (((value >> (i - first)) & 1) == 1)
				buffer[i / 8] |= 1 << (i % 8);
			else
				buffer[i / 8] &= ~(1 << (i % 8));
		}
	}
}

/**
 * Sets @c num bits in @c _buffer, starting at the @c first bit,
 * using the bits in @c value.  This routine fast-paths writes
 * of little-endian, byte-aligned, 64-bit words.
 * @param _buffer The buffer whose bits will be set.
 *	Do not use uninitialized buffer or clang static analyzer emits a warning.
 * @param first The bit offset in @c _buffer to start writing (0-63).
 * @param num The number of bits from @c value to copy (1-64).
 * @param value Up to 64 bits that will be copied to _buffer.
 */
static inline void buf_set_u64(uint8_t *_buffer,
	unsigned first, unsigned num, uint64_t value)
{
	uint8_t *buffer = _buffer;

	if ((num == 32) && (first == 0)) {
		buffer[3] = (value >> 24) & 0xff;
		buffer[2] = (value >> 16) & 0xff;
		buffer[1] = (value >> 8) & 0xff;
		buffer[0] = (value >> 0) & 0xff;
	} else if ((num == 64) && (first == 0)) {
		buffer[7] = (value >> 56) & 0xff;
		buffer[6] = (value >> 48) & 0xff;
		buffer[5] = (value >> 40) & 0xff;
		buffer[4] = (value >> 32) & 0xff;
		buffer[3] = (value >> 24) & 0xff;
		buffer[2] = (value >> 16) & 0xff;
		buffer[1] = (value >> 8) & 0xff;
		buffer[0] = (value >> 0) & 0xff;
	} else {
		for (unsigned i = first; i < first + num; i++) {
			if (((value >> (i - first)) & 1) == 1)
				buffer[i / 8] |= 1 << (i % 8);
			else
				buffer[i / 8] &= ~(1 << (i % 8));
		}
	}
}

/**
 * Retrieves @c num bits from @c _buffer, starting at the @c first bit,
 * returning the bits in a 32-bit word.  This routine fast-paths reads
 * of little-endian, byte-aligned, 32-bit words.
 * @param _buffer The buffer whose bits will be read.
 * @param first The bit offset in @c _buffer to start reading (0-31).
 * @param num The number of bits from @c _buffer to read (1-32).
 * @returns Up to 32-bits that were read from @c _buffer.
 */
static inline uint32_t buf_get_u32(const uint8_t *_buffer,
	unsigned first, unsigned num)
{
	const uint8_t *buffer = _buffer;

	if ((num == 32) && (first == 0)) {
		return (((uint32_t)buffer[3]) << 24) |
				(((uint32_t)buffer[2]) << 16) |
				(((uint32_t)buffer[1]) << 8) |
				(((uint32_t)buffer[0]) << 0);
	} else {
		uint32_t result = 0;
		for (unsigned i = first; i < first + num; i++) {
			if (((buffer[i / 8] >> (i % 8)) & 1) == 1)
				result |= 1U << (i - first);
		}
		return result;
	}
}

/**
 * Retrieves @c num bits from @c _buffer, starting at the @c first bit,
 * returning the bits in a 64-bit word.  This routine fast-paths reads
 * of little-endian, byte-aligned, 64-bit words.
 * @param _buffer The buffer whose bits will be read.
 * @param first The bit offset in @c _buffer to start reading (0-63).
 * @param num The number of bits from @c _buffer to read (1-64).
 * @returns Up to 64-bits that were read from @c _buffer.
 */
static inline uint64_t buf_get_u64(const uint8_t *_buffer,
	unsigned first, unsigned num)
{
	const uint8_t *buffer = _buffer;

	if ((num == 32) && (first == 0)) {
		return 0 + ((((uint32_t)buffer[3]) << 24) |   /* Note - zero plus is to avoid a checkpatch bug */
				(((uint32_t)buffer[2]) << 16) |
				(((uint32_t)buffer[1]) << 8)  |
				(((uint32_t)buffer[0]) << 0));
	} else if ((num == 64) && (first == 0)) {
		return 0 + ((((uint64_t)buffer[7]) << 56) |   /* Note - zero plus is to avoid a checkpatch bug */
				(((uint64_t)buffer[6]) << 48) |
				(((uint64_t)buffer[5]) << 40) |
				(((uint64_t)buffer[4]) << 32) |
				(((uint64_t)buffer[3]) << 24) |
				(((uint64_t)buffer[2]) << 16) |
				(((uint64_t)buffer[1]) << 8)  |
				(((uint64_t)buffer[0]) << 0));
	} else {
		uint64_t result = 0;
		for (unsigned i = first; i < first + num; i++) {
			if (((buffer[i / 8] >> (i % 8)) & 1) == 1)
				result = result | ((uint64_t)1 << (uint64_t)(i - first));
		}
		return result;
	}
}


/**
 * Inverts the ordering of bits inside a 32-bit word (e.g. 31..0 -> 0..31).
 * This routine can be used to flip smaller data types by using smaller
 * values for @c width.
 * @param value The word to flip.
 * @param width The number of bits in value (2-32).
 * @returns A 32-bit word with @c value in reversed bit-order.
 */
uint32_t flip_u32(uint32_t value, unsigned width);

bool buf_cmp(const void *buf1, const void *buf2, unsigned size);
bool buf_cmp_mask(const void *buf1, const void *buf2,
		const void *mask, unsigned size);

/**
 * Copies @c size bits out of @c from and into @c to.  Any extra
 * bits in the final byte will be set to zero.
 * @param from The buffer to copy into @c to.
 * @param to The buffer that will receive the copy of @c from.
 * @param size The number of bits to copy.
 */
void *buf_cpy(const void *from, void *to, unsigned size);

/**
 * Set the contents of @c buf with @c count bits, all set to 1.
 * @param buf The buffer to fill with ones.
 * @param size The number of bits.
 * @returns The original buffer (@c buf).
 */
void *buf_set_ones(void *buf, unsigned size);

void *buf_set_buf(const void *src, unsigned src_start,
		  void *dst, unsigned dst_start, unsigned len);

int str_to_buf(const char *str, unsigned len,
		void *bin_buf, unsigned buf_size, unsigned radix);
char *buf_to_hex_str(const void *buf, unsigned size);

/* read a uint32_t from a buffer in target memory endianness */
static inline uint32_t fast_target_buffer_get_u32(const void *p, bool le)
{
	return le ? le_to_h_u32(p) : be_to_h_u32(p);
}

static inline void bit_copy(uint8_t *dst, unsigned dst_offset, const uint8_t *src,
	unsigned src_offset, unsigned bit_count)
{
	buf_set_buf(src, src_offset, dst, dst_offset, bit_count);
}

struct bit_copy_queue {
	struct list_head list;
};

struct bit_copy_queue_entry {
	uint8_t *dst;
	unsigned dst_offset;
	const uint8_t *src;
	unsigned src_offset;
	unsigned bit_count;
	struct list_head list;
};

void bit_copy_queue_init(struct bit_copy_queue *q);
int bit_copy_queued(struct bit_copy_queue *q, uint8_t *dst, unsigned dst_offset, const uint8_t *src,
		    unsigned src_offset, unsigned bit_count);
void bit_copy_execute(struct bit_copy_queue *q);
void bit_copy_discard(struct bit_copy_queue *q);

/* functions to convert to/from hex encoded buffer
 * used in ti-icdi driver and gdb server */
size_t unhexify(uint8_t *bin, const char *hex, size_t count);
size_t hexify(char *hex, const uint8_t *bin, size_t count, size_t out_maxlen);
void buffer_shr(void *_buf, unsigned buf_len, unsigned count);

#endif /* OPENOCD_HELPER_BINARYBUFFER_H */
