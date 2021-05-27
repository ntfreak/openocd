/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright (C) 2021 by Andreas Fritiofson <andreas.fritiofson@gmail.com> */

/*
 * Simple example of using a circular doubly linked list through list.h
 *
 * gcc -I ../src/ list_example.c -o list_example
 */

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <helper/list.h>

static LIST_HEAD(threads);

struct thread {
	int id;
	uint64_t tcb_address;
	struct list_head lh;
};

void insert(struct thread *t)
{
	list_add_tail(&t->lh, &threads);
}

void remove(struct thread *t)
{
	list_del(&t->lh);
}

struct thread *lookup_id(int id)
{
	struct thread *t;
	list_for_each_entry(t, &threads, lh) {
		if (t->id == id)
			return t;
	}
	return NULL;
}

struct thread *lookup_tcb(uint64_t addr)
{
	struct thread *t;
	list_for_each_entry(t, &threads, lh) {
		if (t->tcb_address == addr)
			return t;
	}
	return NULL;
}

int main(void)
{
	struct thread t1 = { .id = 1, .tcb_address = 111111111 };
	struct thread t2 = { .id = 2, .tcb_address = 222222222 };
	struct thread t3 = { .id = 3, .tcb_address = 333333333 };

	insert(&t1);
	insert(&t2);
	assert(lookup_id(1) == &t1);
	assert(lookup_tcb(111111111) == &t1);
	assert(lookup_id(2) == &t2);
	assert(lookup_id(42) == NULL);
	remove(&t1);
	assert(lookup_id(1) == NULL);
	insert(&t3);
	remove(&t2);
	assert(lookup_id(3) == &t3);
	assert(lookup_tcb(333333333) == &t3);
	assert(lookup_id(2) == NULL);
	remove(&t3);
	assert(list_empty(&threads));
}
