/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
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

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

static int jim_newtap_expected_id(struct jim_nvp *n, struct jim_getopt_info *goi,
				  struct jtag_tap *tap)
{
	jim_wide w;
	int e = jim_getopt_wide(goi, &w);
	if (e != JIM_OK) {
		Jim_SetResultFormatted(goi->interp, "option: %s bad parameter",
				       n->name);
		return e;
	}

	uint32_t *p = realloc(tap->expected_ids,
			      (tap->expected_ids_cnt + 1) * sizeof(uint32_t));
	if (!p) {
		Jim_SetResultFormatted(goi->interp, "no memory");
		return JIM_ERR;
	}

	tap->expected_ids = p;
	tap->expected_ids[tap->expected_ids_cnt++] = w;

	return JIM_OK;
}

#define NTAP_OPT_IRLEN     0
#define NTAP_OPT_IRMASK    1
#define NTAP_OPT_IRCAPTURE 2
#define NTAP_OPT_ENABLED   3
#define NTAP_OPT_DISABLED  4
#define NTAP_OPT_EXPECTED_ID 5
#define NTAP_OPT_VERSION   6

static int jim_hl_newtap_cmd(struct jim_getopt_info *goi)
{
	struct jtag_tap *tap;
	int x;
	int e;
	struct jim_nvp *n;
	char *cp;
	const struct jim_nvp opts[] = {
		{ .name = "-irlen",       .value = NTAP_OPT_IRLEN },
		{ .name = "-irmask",       .value = NTAP_OPT_IRMASK },
		{ .name = "-ircapture",       .value = NTAP_OPT_IRCAPTURE },
		{ .name = "-enable",       .value = NTAP_OPT_ENABLED },
		{ .name = "-disable",       .value = NTAP_OPT_DISABLED },
		{ .name = "-expected-id",       .value = NTAP_OPT_EXPECTED_ID },
		{ .name = "-ignore-version",       .value = NTAP_OPT_VERSION },
		{ .name = NULL, .value = -1},
	};

	tap = calloc(1, sizeof(struct jtag_tap));
	if (!tap) {
		Jim_SetResultFormatted(goi->interp, "no memory");
		return JIM_ERR;
	}

	/*
	 * we expect CHIP + TAP + OPTIONS
	 * */
	if (goi->argc < 3) {
		Jim_SetResultFormatted(goi->interp,
				       "Missing CHIP TAP OPTIONS ....");
		free(tap);
		return JIM_ERR;
	}

	const char *tmp;
	jim_getopt_string(goi, &tmp, NULL);
	tap->chip = strdup(tmp);

	jim_getopt_string(goi, &tmp, NULL);
	tap->tapname = strdup(tmp);

	/* name + dot + name + null */
	x = strlen(tap->chip) + 1 + strlen(tap->tapname) + 1;
	cp = malloc(x);
	sprintf(cp, "%s.%s", tap->chip, tap->tapname);
	tap->dotted_name = cp;

	LOG_DEBUG("Creating New Tap, Chip: %s, Tap: %s, Dotted: %s, %d params",
		  tap->chip, tap->tapname, tap->dotted_name, goi->argc);

	while (goi->argc) {
		e = jim_getopt_nvp(goi, opts, &n);
		if (e != JIM_OK) {
			jim_getopt_nvp_unknown(goi, opts, 0);
			free(cp);
			free(tap);
			return e;
		}
		LOG_DEBUG("Processing option: %s", n->name);
		switch (n->value) {
		case NTAP_OPT_EXPECTED_ID:
			e = jim_newtap_expected_id(n, goi, tap);
			if (e != JIM_OK) {
				free(cp);
				free(tap);
				return e;
			}
			break;
		case NTAP_OPT_IRLEN:
		case NTAP_OPT_IRMASK:
		case NTAP_OPT_IRCAPTURE:
			/* dummy read to ignore the next argument */
			jim_getopt_wide(goi, NULL);
			break;
		}		/* switch (n->value) */
	}			/* while (goi->argc) */

	/* default is enabled-after-reset */
	tap->enabled = !tap->disabled_after_reset;

	jtag_tap_init(tap);
	return JIM_OK;
}

int jim_hl_newtap(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc - 1, argv + 1);
	return jim_hl_newtap_cmd(&goi);
}
