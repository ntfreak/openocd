/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2018 by Advantest                                       *
 *   florian.meister@advantest.com                                         *
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

#ifndef OPENOCD_TARGET_IMAGE_H
#define OPENOCD_TARGET_IMAGE_H

#include <helper/fileio.h>

#ifdef HAVE_ELF_H
#include <elf.h>
#endif

#define IMAGE_MAX_ERROR_STRING		(256)
#define IMAGE_MAX_SECTIONS			(512)

#define IMAGE_MEMORY_CACHE_SIZE		(2048)

enum image_type {
	IMAGE_BINARY,	/* plain binary */
	IMAGE_IHEX,		/* intel hex-record format */
	IMAGE_MEMORY,	/* target-memory pseudo-image */
	IMAGE_ELF,		/* ELF binary */
	IMAGE_SRECORD,	/* motorola s19 */
	IMAGE_BUILDER,	/* when building a new image */
};

struct imagesection {
	target_addr_t base_address;
	uint32_t size;
	int flags;
	void *private;		/* private data */
};

struct image {
	enum image_type type;		/* image type (plain, ihex, ...) */
	void *type_private;		/* type private data */
	unsigned int num_sections;		/* number of sections contained in the image */
	struct imagesection *sections;	/* array of sections */
	bool base_address_set;	/* whether the image has a base address set (for relocation purposes) */
	long long base_address;		/* base address, if one is set */
	bool start_address_set;	/* whether the image has a start address (entry point) associated */
	uint32_t start_address;		/* start address, if one is set */
};

struct image_binary {
	struct fileio *fileio;
};

struct image_ihex {
	struct fileio *fileio;
	uint8_t *buffer;
};

struct image_memory {
	struct target *target;
	uint8_t *cache;
	uint32_t cache_address;
};

struct image_elf {
	struct fileio *fileio;
	bool is_64_bit;
	union {
		Elf32_Ehdr *header32;
		Elf64_Ehdr *header64;
	};
	union {
		Elf32_Phdr *segments32;
		Elf64_Phdr *segments64;
	};
	uint32_t segment_count;
	uint8_t endianness;
};

struct image_mot {
	struct fileio *fileio;
	uint8_t *buffer;
};

int image_open(struct image *image, const char *url, const char *type_string);
int image_read_section(struct image *image, int section, target_addr_t offset,
		uint32_t size, uint8_t *buffer, size_t *size_read);
void image_close(struct image *image);

int image_add_section(struct image *image, target_addr_t base, uint32_t size,
		int flags, uint8_t const *data);

int image_calculate_checksum(const uint8_t *buffer, uint32_t nbytes,
		uint32_t *checksum);

#define ERROR_IMAGE_FORMAT_ERROR	(-1400)
#define ERROR_IMAGE_TYPE_UNKNOWN	(-1401)
#define ERROR_IMAGE_TEMPORARILY_UNAVAILABLE		(-1402)
#define ERROR_IMAGE_CHECKSUM		(-1403)

#endif /* OPENOCD_TARGET_IMAGE_H */
