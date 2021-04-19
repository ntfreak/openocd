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

#ifndef OPENOCD_HELPER_FILEIO_H
#define OPENOCD_HELPER_FILEIO_H

#include "types.h"

#define FILEIO_MAX_ERROR_STRING		(128)

enum fileio_type {
	FILEIO_TEXT,
	FILEIO_BINARY,
};

enum fileio_access {
	FILEIO_NONE,		/* open without any access (invalid mode) */
	FILEIO_READ,		/* open for reading, position at beginning */
	FILEIO_WRITE,		/* open for writing, position at beginning */
	FILEIO_READWRITE,	/* open for writing, position at beginning, allow reading */
	FILEIO_APPEND,		/* open for writing, position at end */
	FILEIO_APPENDREAD,	/* open for writing, position at end, allow reading */
};

struct fileio;

int fileio_open(struct fileio **fileio, const char *url,
		enum fileio_access access_type, enum fileio_type type);
int fileio_close(struct fileio *fileio);
int fileio_feof(struct fileio *fileio);

int fileio_seek(struct fileio *fileio, size_t position);
int fileio_fgets(struct fileio *fileio, size_t size, void *buffer);

int fileio_read(struct fileio *fileio,
		size_t size, void *buffer, size_t *size_read);
int fileio_write(struct fileio *fileio,
		size_t size, const void *buffer, size_t *size_written);

int fileio_read_u32(struct fileio *fileio, uint32_t *data);
int fileio_write_u32(struct fileio *fileio, uint32_t data);
int fileio_size(struct fileio *fileio, size_t *size);

#define ERROR_FILEIO_LOCATION_UNKNOWN			(-1200)
#define ERROR_FILEIO_NOT_FOUND					(-1201)
#define ERROR_FILEIO_OPERATION_FAILED			(-1202)
#define ERROR_FILEIO_ACCESS_NOT_SUPPORTED		(-1203)
#define ERROR_FILEIO_RESOURCE_TYPE_UNKNOWN		(-1204)
#define ERROR_FILEIO_OPERATION_NOT_SUPPORTED	(-1205)

#endif /* OPENOCD_HELPER_FILEIO_H */
