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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "time_support.h"

/* calculate difference between two struct timeval values */
int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y)
{
	if (x->tv_usec < y->tv_usec) {
		int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
		y->tv_usec -= 1000000 * nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_usec - y->tv_usec > 1000000) {
		int nsec = (x->tv_usec - y->tv_usec) / 1000000;
		y->tv_usec += 1000000 * nsec;
		y->tv_sec -= nsec;
	}

	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
}

int timeval_add_time(struct timeval *result, long sec, long usec)
{
	result->tv_sec += sec;
	result->tv_usec += usec;

	while (result->tv_usec > 1000000) {
		result->tv_usec -= 1000000;
		result->tv_sec++;
	}

	return 0;
}

/* compare two timevals and return -1/0/+1 accordingly */
int timeval_compare(const struct timeval *x, const struct timeval *y)
{
	if (x->tv_sec < y->tv_sec)
		return -1;
	else if (x->tv_sec > y->tv_sec)
		return 1;
	else if (x->tv_usec < y->tv_usec)
		return -1;
	else if (x->tv_usec > y->tv_usec)
		return 1;
	else
		return 0;
}

int duration_start(struct duration *duration)
{
	return gettimeofday(&duration->start, NULL);
}

int duration_measure(struct duration *duration)
{
	struct timeval end;
	int retval = gettimeofday(&end, NULL);
	if (retval == 0)
		timeval_subtract(&duration->elapsed, &end, &duration->start);
	return retval;
}

float duration_elapsed(const struct duration *duration)
{
	float t = duration->elapsed.tv_sec;
	t += (float)duration->elapsed.tv_usec / 1000000.0;
	return t;
}

float duration_kbps(const struct duration *duration, size_t count)
{
	return count / (1024.0 * duration_elapsed(duration));
}
