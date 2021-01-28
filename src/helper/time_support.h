/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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

#ifndef OPENOCD_HELPER_TIME_SUPPORT_H
#define OPENOCD_HELPER_TIME_SUPPORT_H

#include <time.h>
#include "types.h"

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y);
int timeval_add_time(struct timeval *result, long sec, long usec);
int timeval_compare(const struct timeval *x, const struct timeval *y);

/** @returns gettimeofday() timeval as 64-bit in ms */
int64_t timeval_ms(void);

struct duration {
	struct timeval start;
	struct timeval elapsed;
};

/** Update the duration->start field to start the @a duration measurement. */
int duration_start(struct duration *duration);
/** Update the duration->elapsed field to finish the @a duration measurement. */
int duration_measure(struct duration *duration);

/** @returns Elapsed time in seconds. */
float duration_elapsed(const struct duration *duration);
/** @returns KB/sec for the elapsed @a duration and @a count bytes. */
float duration_kbps(const struct duration *duration, size_t count);

#endif /* OPENOCD_HELPER_TIME_SUPPORT_H */
