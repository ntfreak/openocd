/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2020 by Tarek Bochkati for STMicroelectronics           *
 *   tarek.bouchkati@gmail.com                                             *
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

#ifndef OPENOCD_HELPER_INTERVAL_H
#define OPENOCD_HELPER_INTERVAL_H

typedef struct interval {
	int start, end;
	struct interval *next;
} interval_t;

int interval_count(interval_t *head);
int interval_append(interval_t **head_ref, int start, int end);
int interval_delete(interval_t **head_ref, int start, int end);
int interval_reorder(interval_t **head_ref);
void interval_print(interval_t *interval);
void interval_print_all(interval_t *head);


#endif /* OPENOCD_HELPER_INTERVAL_H */
