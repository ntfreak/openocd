/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) ST-Ericsson SA 2011                                     *
 *   michel.jaouen@stericsson.com : smp minimum support                    *
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

#include "target.h"
#include <helper/log.h>
#include "breakpoints.h"

static const char * const breakpoint_type_strings[] = {
	"hardware",
	"software"
};

static const char * const watchpoint_rw_strings[] = {
	"read",
	"write",
	"access"
};

/* monotonic counter/id-number for breakpoints and watch points */
static int bpwp_unique_id;

static int breakpoint_add_internal(struct target *target,
	target_addr_t address,
	uint32_t length,
	enum breakpoint_type type)
{
	struct breakpoint *breakpoint = target->breakpoints;
	struct breakpoint **breakpoint_p = &target->breakpoints;
	const char *reason;
	int retval;

	while (breakpoint) {
		if (breakpoint->address == address) {
			/* FIXME don't assume "same address" means "same
			 * breakpoint" ... check all the parameters before
			 * succeeding.
			 */
			LOG_ERROR("Duplicate Breakpoint address: " TARGET_ADDR_FMT " (BP %" PRIu32 ")",
				address, breakpoint->unique_id);
			return ERROR_TARGET_DUPLICATE_BREAKPOINT;
		}
		breakpoint_p = &breakpoint->next;
		breakpoint = breakpoint->next;
	}

	(*breakpoint_p) = malloc(sizeof(struct breakpoint));
	(*breakpoint_p)->address = address;
	(*breakpoint_p)->asid = 0;
	(*breakpoint_p)->length = length;
	(*breakpoint_p)->type = type;
	(*breakpoint_p)->set = 0;
	(*breakpoint_p)->orig_instr = malloc(length);
	(*breakpoint_p)->next = NULL;
	(*breakpoint_p)->unique_id = bpwp_unique_id++;

	retval = target_add_breakpoint(target, *breakpoint_p);
	switch (retval) {
		case ERROR_OK:
			break;
		case ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
			reason = "resource not available";
			goto fail;
		case ERROR_TARGET_NOT_HALTED:
			reason = "target running";
			goto fail;
		default:
			reason = "unknown reason";
fail:
			LOG_ERROR("can't add breakpoint: %s", reason);
			free((*breakpoint_p)->orig_instr);
			free(*breakpoint_p);
			*breakpoint_p = NULL;
			return retval;
	}

	LOG_DEBUG("added %s breakpoint at " TARGET_ADDR_FMT " of length 0x%8.8x, (BPID: %" PRIu32 ")",
		breakpoint_type_strings[(*breakpoint_p)->type],
		(*breakpoint_p)->address, (*breakpoint_p)->length,
		(*breakpoint_p)->unique_id);

	return ERROR_OK;
}

static int context_breakpoint_add_internal(struct target *target,
	uint32_t asid,
	uint32_t length,
	enum breakpoint_type type)
{
	struct breakpoint *breakpoint = target->breakpoints;
	struct breakpoint **breakpoint_p = &target->breakpoints;
	int retval;

	while (breakpoint) {
		if (breakpoint->asid == asid) {
			/* FIXME don't assume "same address" means "same
			 * breakpoint" ... check all the parameters before
			 * succeeding.
			 */
			LOG_ERROR("Duplicate Breakpoint asid: 0x%08" PRIx32 " (BP %" PRIu32 ")",
				asid, breakpoint->unique_id);
			return ERROR_TARGET_DUPLICATE_BREAKPOINT;
		}
		breakpoint_p = &breakpoint->next;
		breakpoint = breakpoint->next;
	}

	(*breakpoint_p) = malloc(sizeof(struct breakpoint));
	(*breakpoint_p)->address = 0;
	(*breakpoint_p)->asid = asid;
	(*breakpoint_p)->length = length;
	(*breakpoint_p)->type = type;
	(*breakpoint_p)->set = 0;
	(*breakpoint_p)->orig_instr = malloc(length);
	(*breakpoint_p)->next = NULL;
	(*breakpoint_p)->unique_id = bpwp_unique_id++;
	retval = target_add_context_breakpoint(target, *breakpoint_p);
	if (retval != ERROR_OK) {
		LOG_ERROR("could not add breakpoint");
		free((*breakpoint_p)->orig_instr);
		free(*breakpoint_p);
		*breakpoint_p = NULL;
		return retval;
	}

	LOG_DEBUG("added %s Context breakpoint at 0x%8.8" PRIx32 " of length 0x%8.8x, (BPID: %" PRIu32 ")",
		breakpoint_type_strings[(*breakpoint_p)->type],
		(*breakpoint_p)->asid, (*breakpoint_p)->length,
		(*breakpoint_p)->unique_id);

	return ERROR_OK;
}

static int hybrid_breakpoint_add_internal(struct target *target,
	target_addr_t address,
	uint32_t asid,
	uint32_t length,
	enum breakpoint_type type)
{
	struct breakpoint *breakpoint = target->breakpoints;
	struct breakpoint **breakpoint_p = &target->breakpoints;
	int retval;

	while (breakpoint) {
		if ((breakpoint->asid == asid) && (breakpoint->address == address)) {
			/* FIXME don't assume "same address" means "same
			 * breakpoint" ... check all the parameters before
			 * succeeding.
			 */
			LOG_ERROR("Duplicate Hybrid Breakpoint asid: 0x%08" PRIx32 " (BP %" PRIu32 ")",
				asid, breakpoint->unique_id);
			return ERROR_TARGET_DUPLICATE_BREAKPOINT;
		} else if ((breakpoint->address == address) && (breakpoint->asid == 0)) {
			LOG_ERROR("Duplicate Breakpoint IVA: " TARGET_ADDR_FMT " (BP %" PRIu32 ")",
				address, breakpoint->unique_id);
			return ERROR_TARGET_DUPLICATE_BREAKPOINT;

		}
		breakpoint_p = &breakpoint->next;
		breakpoint = breakpoint->next;
	}
	(*breakpoint_p) = malloc(sizeof(struct breakpoint));
	(*breakpoint_p)->address = address;
	(*breakpoint_p)->asid = asid;
	(*breakpoint_p)->length = length;
	(*breakpoint_p)->type = type;
	(*breakpoint_p)->set = 0;
	(*breakpoint_p)->orig_instr = malloc(length);
	(*breakpoint_p)->next = NULL;
	(*breakpoint_p)->unique_id = bpwp_unique_id++;


	retval = target_add_hybrid_breakpoint(target, *breakpoint_p);
	if (retval != ERROR_OK) {
		LOG_ERROR("could not add breakpoint");
		free((*breakpoint_p)->orig_instr);
		free(*breakpoint_p);
		*breakpoint_p = NULL;
		return retval;
	}
	LOG_DEBUG(
		"added %s Hybrid breakpoint at address " TARGET_ADDR_FMT " of length 0x%8.8x, (BPID: %" PRIu32 ")",
		breakpoint_type_strings[(*breakpoint_p)->type],
		(*breakpoint_p)->address,
		(*breakpoint_p)->length,
		(*breakpoint_p)->unique_id);

	return ERROR_OK;
}

int breakpoint_add(struct target *target,
	target_addr_t address,
	uint32_t length,
	enum breakpoint_type type)
{
	int retval = ERROR_OK;
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		head = target->head;
		if (type == BKPT_SOFT)
			return breakpoint_add_internal(head->target, address, length, type);

		while (head != (struct target_list *)NULL) {
			curr = head->target;
			retval = breakpoint_add_internal(curr, address, length, type);
			if (retval != ERROR_OK)
				return retval;
			head = head->next;
		}
		return retval;
	} else {
		return breakpoint_add_internal(target, address, length, type);
	}
}

int context_breakpoint_add(struct target *target,
	uint32_t asid,
	uint32_t length,
	enum breakpoint_type type)
{
	int retval = ERROR_OK;
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			retval = context_breakpoint_add_internal(curr, asid, length, type);
			if (retval != ERROR_OK)
				return retval;
			head = head->next;
		}
		return retval;
	} else {
		return context_breakpoint_add_internal(target, asid, length, type);
	}
}

int hybrid_breakpoint_add(struct target *target,
	target_addr_t address,
	uint32_t asid,
	uint32_t length,
	enum breakpoint_type type)
{
	int retval = ERROR_OK;
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			retval = hybrid_breakpoint_add_internal(curr, address, asid, length, type);
			if (retval != ERROR_OK)
				return retval;
			head = head->next;
		}
		return retval;
	} else
		return hybrid_breakpoint_add_internal(target, address, asid, length, type);
}

/* free up a breakpoint */
static void breakpoint_free(struct target *target, struct breakpoint *breakpoint_to_remove)
{
	struct breakpoint *breakpoint = target->breakpoints;
	struct breakpoint **breakpoint_p = &target->breakpoints;
	int retval;

	while (breakpoint) {
		if (breakpoint == breakpoint_to_remove)
			break;
		breakpoint_p = &breakpoint->next;
		breakpoint = breakpoint->next;
	}

	if (!breakpoint)
		return;

	retval = target_remove_breakpoint(target, breakpoint);

	LOG_DEBUG("free BPID: %" PRIu32 " --> %d", breakpoint->unique_id, retval);
	(*breakpoint_p) = breakpoint->next;
	free(breakpoint->orig_instr);
	free(breakpoint);
}

static int breakpoint_remove_internal(struct target *target, target_addr_t address)
{
	struct breakpoint *breakpoint = target->breakpoints;

	while (breakpoint) {
		if ((breakpoint->address == address) ||
		    (breakpoint->address == 0 && breakpoint->asid == address))
			break;
		breakpoint = breakpoint->next;
	}

	if (breakpoint) {
		breakpoint_free(target, breakpoint);
		return 1;
	} else {
		if (!target->smp)
			LOG_ERROR("no breakpoint at address " TARGET_ADDR_FMT " found", address);
		return 0;
	}
}

static void breakpoint_remove_all_internal(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	while (breakpoint) {
		struct breakpoint *tmp = breakpoint;
		breakpoint = breakpoint->next;
		breakpoint_free(target, tmp);
	}
}

void breakpoint_remove(struct target *target, target_addr_t address)
{
	if (target->smp) {
		unsigned int num_breakpoints = 0;
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			num_breakpoints += breakpoint_remove_internal(curr, address);
			head = head->next;
		}
		if (!num_breakpoints)
			LOG_ERROR("no breakpoint at address " TARGET_ADDR_FMT " found", address);
	} else {
		breakpoint_remove_internal(target, address);
	}
}

void breakpoint_remove_all(struct target *target)
{
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			breakpoint_remove_all_internal(curr);
			head = head->next;
		}
	} else {
		breakpoint_remove_all_internal(target);
	}
}

static void breakpoint_clear_target_internal(struct target *target)
{
	LOG_DEBUG("Delete all breakpoints for target: %s",
		target_name(target));
	while (target->breakpoints)
		breakpoint_free(target, target->breakpoints);
}

void breakpoint_clear_target(struct target *target)
{
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			breakpoint_clear_target_internal(curr);
			head = head->next;
		}
	} else {
		breakpoint_clear_target_internal(target);
	}
}

struct breakpoint *breakpoint_find(struct target *target, target_addr_t address)
{
	struct breakpoint *breakpoint = target->breakpoints;

	while (breakpoint) {
		if (breakpoint->address == address)
			return breakpoint;
		breakpoint = breakpoint->next;
	}

	return NULL;
}

int watchpoint_add(struct target *target, target_addr_t address, uint32_t length,
	enum watchpoint_rw rw, uint32_t value, uint32_t mask)
{
	struct watchpoint *watchpoint = target->watchpoints;
	struct watchpoint **watchpoint_p = &target->watchpoints;
	int retval;
	const char *reason;

	while (watchpoint) {
		if (watchpoint->address == address) {
			if (watchpoint->length != length
				|| watchpoint->value != value
				|| watchpoint->mask != mask
				|| watchpoint->rw != rw) {
				LOG_ERROR("address " TARGET_ADDR_FMT
					" already has watchpoint %d",
					address, watchpoint->unique_id);
				return ERROR_FAIL;
			}

			/* ignore duplicate watchpoint */
			return ERROR_OK;
		}
		watchpoint_p = &watchpoint->next;
		watchpoint = watchpoint->next;
	}

	(*watchpoint_p) = calloc(1, sizeof(struct watchpoint));
	(*watchpoint_p)->address = address;
	(*watchpoint_p)->length = length;
	(*watchpoint_p)->value = value;
	(*watchpoint_p)->mask = mask;
	(*watchpoint_p)->rw = rw;
	(*watchpoint_p)->unique_id = bpwp_unique_id++;

	retval = target_add_watchpoint(target, *watchpoint_p);
	switch (retval) {
		case ERROR_OK:
			break;
		case ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
			reason = "resource not available";
			goto bye;
		case ERROR_TARGET_NOT_HALTED:
			reason = "target running";
			goto bye;
		default:
			reason = "unrecognized error";
bye:
			LOG_ERROR("can't add %s watchpoint at " TARGET_ADDR_FMT ", %s",
				watchpoint_rw_strings[(*watchpoint_p)->rw],
				address, reason);
			free(*watchpoint_p);
			*watchpoint_p = NULL;
			return retval;
	}

	LOG_DEBUG("added %s watchpoint at " TARGET_ADDR_FMT
		" of length 0x%8.8" PRIx32 " (WPID: %d)",
		watchpoint_rw_strings[(*watchpoint_p)->rw],
		(*watchpoint_p)->address,
		(*watchpoint_p)->length,
		(*watchpoint_p)->unique_id);

	return ERROR_OK;
}

static void watchpoint_free(struct target *target, struct watchpoint *watchpoint_to_remove)
{
	struct watchpoint *watchpoint = target->watchpoints;
	struct watchpoint **watchpoint_p = &target->watchpoints;
	int retval;

	while (watchpoint) {
		if (watchpoint == watchpoint_to_remove)
			break;
		watchpoint_p = &watchpoint->next;
		watchpoint = watchpoint->next;
	}

	if (!watchpoint)
		return;
	retval = target_remove_watchpoint(target, watchpoint);
	LOG_DEBUG("free WPID: %d --> %d", watchpoint->unique_id, retval);
	(*watchpoint_p) = watchpoint->next;
	free(watchpoint);
}

void watchpoint_remove(struct target *target, target_addr_t address)
{
	struct watchpoint *watchpoint = target->watchpoints;

	while (watchpoint) {
		if (watchpoint->address == address)
			break;
		watchpoint = watchpoint->next;
	}

	if (watchpoint)
		watchpoint_free(target, watchpoint);
	else
		LOG_ERROR("no watchpoint at address " TARGET_ADDR_FMT " found", address);
}

void watchpoint_clear_target(struct target *target)
{
	LOG_DEBUG("Delete all watchpoints for target: %s",
		target_name(target));
	while (target->watchpoints)
		watchpoint_free(target, target->watchpoints);
}

int watchpoint_hit(struct target *target, enum watchpoint_rw *rw,
		   target_addr_t *address)
{
	int retval;
	struct watchpoint *hit_watchpoint;

	retval = target_hit_watchpoint(target, &hit_watchpoint);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	*rw = hit_watchpoint->rw;
	*address = hit_watchpoint->address;

	LOG_DEBUG("Found hit watchpoint at " TARGET_ADDR_FMT " (WPID: %d)",
		hit_watchpoint->address,
		hit_watchpoint->unique_id);

	return ERROR_OK;
}
