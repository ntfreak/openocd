/*
 * Copyright (C) 2020 by Tarek Bochkati for STMicroelectronics           *
 * tarek.bouchkati@gmail.com                                             *
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

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
void interval_destroy(interval_t *head);
void interval_print_all(interval_t *head, char *str);


#endif /* OPENOCD_HELPER_INTERVAL_H */
