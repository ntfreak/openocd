/*
 * Copyright (C) 2020 by Tarek Bochkati for STMicroelectronics           *
 * tarek.bouchkati@gmail.com                                             *
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef OPENOCD_HELPER_INTERVAL_H
#define OPENOCD_HELPER_INTERVAL_H

struct interval {
	int start, end;
	struct interval *next;
};

int interval_count(struct interval *head);
int interval_append(struct interval **head_ref, int start, int end);
int interval_delete(struct interval **head_ref, int start, int end);
int interval_reorder(struct interval **head_ref);
void interval_destroy(struct interval *head);
void interval_print_all(struct interval *head, char *str);


#endif /* OPENOCD_HELPER_INTERVAL_H */
