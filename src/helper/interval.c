/*
 * Copyright (C) 2020 by Tarek Bochkati for STMicroelectronics           *
 * tarek.bouchkati@gmail.com                                             *
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "interval.h"

#include <stdlib.h>
#include "log.h"

/* this enum is used to compare two intervals
 *  - interval1 defined by [s1, e1]  (s1 <= e1)
 *  - interval2 defined by [s2, e2]  (s2 <= e2)
 */
enum interval_comparison_result {
	INTERVAL_EQUAL,               /* intervals are equal */
	INTERVAL_BEFORE,              /* [s1 <= e1] < [s2 <= e2] */
	INTERVAL_AFTER,               /* [s2 <= e2] < [s1 <= e1] */
	INTERVAL_BEFORE_WITH_OVERLAP, /* s1 < [s2 <= e1 <= e2] */
	INTERVAL_AFTER_WITH_OVERLAP,  /* [s2 <= s1 <= e2] < e1 */
	INTERVAL_INTO,                /* s2 <= [s1 <= e1] <= e2 */
	INTERVAL_EXTO                 /* s1 < [s2 <= e2] < e1, (not sure about the name) */
};

int interval_count(interval_t *head)
{
	interval_t *cur = head;
	int count = 0;

	while (cur) {
		cur = cur->next;
		count++;
	}

	return count;
}

int interval_append(interval_t **head_ref, int start, int end)
{
	if (start > end) {
		LOG_ERROR("interval error: start > end");
		return ERROR_FAIL;
	}

	interval_t *new_interval = malloc(sizeof(interval_t));
	if (!new_interval)
		return ERROR_FAIL;

	new_interval->start = start;
	new_interval->end = end;
	new_interval->next = NULL;

	if (*head_ref) {
		interval_t *last = *head_ref;

		while (last->next)
			last = last->next;

		last->next = new_interval;
	} else {
		/* first insertion */
		*head_ref = new_interval;
	}

	return ERROR_OK;
}

static enum interval_comparison_result interval_compare(interval_t *interval1, interval_t *interval2)
{
	if ((interval1->start == interval2->start) && (interval1->end == interval2->end))
		return INTERVAL_EQUAL;

	if (interval1->start < interval2->start) {
		if (interval1->end < interval2->start)
			return INTERVAL_BEFORE;
		else if (interval1->end > interval2->end)
			return INTERVAL_EXTO;
		return INTERVAL_BEFORE_WITH_OVERLAP;
	} else if (interval1->start <= interval2->end) {
		if (interval1->end <= interval2->end)
			return INTERVAL_INTO;
		else
			return INTERVAL_AFTER_WITH_OVERLAP;
	}

	return INTERVAL_AFTER;
}

int interval_delete(interval_t **head_ref, int del_start, int del_end)
{
	interval_t *cur = *head_ref, *prev = NULL, *tmp;
	interval_t del_interval = {del_start, del_end, NULL}; /* the interval to delete */

	while (cur) {
		switch (interval_compare(cur, &del_interval)) {
		case INTERVAL_BEFORE:
		case INTERVAL_AFTER:
			/* nothing to do */
			break;
		case INTERVAL_INTO:
		case INTERVAL_EQUAL:
			cur->start = cur->end + 1; /* just a hint to remove the interval */
			break;
		case INTERVAL_BEFORE_WITH_OVERLAP:
			cur->end = del_start - 1; /* check later the interval validity */
			break;
		case INTERVAL_AFTER_WITH_OVERLAP:
			cur->start = del_end + 1; /* check later the interval validity */
			break;
		case INTERVAL_EXTO:
			/* split cur */
			tmp = malloc(sizeof(interval_t));
			if (!tmp)
				return ERROR_FAIL;
			tmp->start = del_end + 1;
			tmp->end = cur->end;
			tmp->next = cur->next;
			cur->end = del_start - 1;
			cur->next = tmp;
			/* jump to next (tmp) */
			cur = cur->next;
			break;
		}

		/* check the cur interval validity */
		if (cur->start > cur->end) {
			/* invalid, remove it */
			if (prev == NULL) {
				/* cur is the first element in the list */
				*head_ref = cur->next;
				free(cur);
				cur = *head_ref;
			} else {
				prev->next = cur->next;
				free(cur);
				cur = prev->next;
			}
		} else if (cur) {
			/* valid, normal execution */
			prev = cur;
			cur = cur->next;
		}
	}

	return ERROR_OK;
}

/* seek for the first occurrences of interval1 and 2 into the list defined by
 * head_ref and swap them */
static int interval_swap(interval_t **head_ref, interval_t *interval1, interval_t *interval2)
{
	assert(interval1 && interval2);

	if (interval1 == interval2) /* nothing to do */
		return ERROR_OK;

	interval_t *tmp = *head_ref;
	interval_t *prev1 = NULL;
	interval_t *prev2 = NULL;

	/* search for prev1 */
	if (*head_ref != interval1) {
		while (tmp && !prev1) {
			if (tmp->next == interval1)
				prev1 = tmp;
			tmp = tmp->next;
		}
		if (!prev1)
			return ERROR_FAIL;
	}

	/* search for prev2 */
	tmp = *head_ref;
	if (*head_ref != interval2) {
		while (tmp && !prev2) {
			if (tmp->next == interval2)
				prev2 = tmp;
			tmp = tmp->next;
		}
		if (!prev2)
			return ERROR_FAIL;
	}

	/* change prev relation chain */
	if (prev1)
		prev1->next = interval2;
	else /* interval1 is the first element, put interval2 instead */
		*head_ref = interval2;

	if (prev2)
		prev2->next = interval1;
	else /* interval2 is the first element, put interval1 instead */
		*head_ref = interval1;

	/* change next relation chain */
	tmp = interval1->next;
	interval1->next = interval2->next;
	interval2->next = tmp;

	return ERROR_OK;
}

int interval_reorder(interval_t **head_ref)
{
	if (interval_count(*head_ref) < 2)
		return ERROR_OK;

	interval_t *cur, *next;
	bool touched;

	/* bubble like algorithm */
	do {
		touched = false;
		cur = *head_ref;

		while (cur && cur->next) {
			next = cur->next;

			enum interval_comparison_result cmp_result = interval_compare(cur, next);

			/* cheat to merge adjacent intervals */
			if (cmp_result == INTERVAL_BEFORE && (cur->end + 1 == next->start))
				cmp_result = INTERVAL_BEFORE_WITH_OVERLAP;

			switch (cmp_result) {
			case INTERVAL_BEFORE:
				/* normal order, nothing to do unless if the intervals are adjacent */
				break;
			case INTERVAL_BEFORE_WITH_OVERLAP:
				/* merge into cur and remove next */
				cur->end = next->end;
				cur->next = next->next;
				free(next);
				touched = true;
				break;
			case INTERVAL_EQUAL:
			case INTERVAL_EXTO:
				/* next is into cur, remove next */
				cur->next = next->next;
				free(next);
				touched = true;
				break;
			case INTERVAL_AFTER_WITH_OVERLAP:
				/* merge into cur and remove next */
				cur->start = next->start;
				cur->next = next->next;
				free(next);
				touched = true;
				break;
			case INTERVAL_INTO:
			case INTERVAL_AFTER:
				if (interval_swap(head_ref, cur, next) != ERROR_OK)
					return ERROR_FAIL;
				touched = true;
				break;
			}

			if (cur)
				cur = cur->next;
		}
	} while (touched);

	return ERROR_OK;
}

void interval_destroy(interval_t *head)
{
	interval_t *cur = head, *next;

	while (cur) {
		next = cur->next;
		free(cur);
		cur = next;
	}
}

static int interval_print_one(interval_t *interval, char *str)
{
	if (interval->start == interval->end)
		return sprintf(str, "[%d]", interval->start);

	return sprintf(str, "[%d,%d]", interval->start, interval->end);
}

void interval_print_all(interval_t *head, char *str)
{
	interval_t *cur = head;
	char *ptr = str;

	/* set str to zero-length string in case of head is null */
	str[0] = '\0';

	while (cur) {
		ptr += interval_print_one(cur, ptr);
		cur = cur->next;
	}
}

