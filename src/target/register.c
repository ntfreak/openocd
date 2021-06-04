/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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

#include "register.h"
#include <helper/log.h>

/**
 * @file
 * Holds utilities to work with register caches.
 *
 * OpenOCD uses machine registers internally, and exposes them by name
 * to Tcl scripts.  Sets of related registers are grouped into caches.
 * For example, a CPU core will expose a set of registers, and there
 * may be separate registers associated with debug or trace modules.
 */

struct reg *register_get_by_number(struct reg_cache *first,
		uint32_t reg_num, bool search_all)
{
	struct reg_cache *cache = first;

	while (cache) {
		for (unsigned int i = 0; i < cache->num_regs; i++) {
			if (!cache->reg_list[i].exist)
				continue;
			if (cache->reg_list[i].number == reg_num)
				return &(cache->reg_list[i]);
		}

		if (!search_all)
			break;

		cache = cache->next;
	}

	return NULL;
}

struct reg *register_get_by_name(struct reg_cache *first,
		const char *name, bool search_all)
{
	struct reg_cache *cache = first;

	while (cache) {
		for (unsigned int i = 0; i < cache->num_regs; i++) {
			if (!cache->reg_list[i].exist)
				continue;
			if (strcmp(cache->reg_list[i].name, name) == 0)
				return &(cache->reg_list[i]);
		}

		if (!search_all)
			break;

		cache = cache->next;
	}

	return NULL;
}

struct reg_cache **register_get_last_cache_p(struct reg_cache **first)
{
	struct reg_cache **cache_p = first;

	if (*cache_p)
		while (*cache_p)
			cache_p = &((*cache_p)->next);
	else
		return first;

	return cache_p;
}

void register_unlink_cache(struct reg_cache **cache_p, const struct reg_cache *cache)
{
	while (*cache_p && *cache_p != cache)
		cache_p = &((*cache_p)->next);
	if (*cache_p)
		*cache_p = cache->next;
}

/** Marks the contents of the register cache as invalid (and clean). */
void register_cache_invalidate(struct reg_cache *cache)
{
	struct reg *reg = cache->reg_list;

	for (unsigned int n = cache->num_regs; n != 0; n--, reg++) {
		if (!reg->exist)
			continue;
		reg->valid = false;
		reg->dirty = false;
	}
}

static int register_get_dummy_core_reg(struct reg *reg)
{
	return ERROR_OK;
}

static int register_set_dummy_core_reg(struct reg *reg, uint8_t *buf)
{
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static const struct reg_arch_type dummy_type = {
	.get = register_get_dummy_core_reg,
	.set = register_set_dummy_core_reg,
};

void register_init_dummy(struct reg *reg)
{
	reg->type = &dummy_type;
}
