/***************************************************************************
 *   Copyright (C) 2011 by Julius Baxter                                   *
 *   julius@opencores.org                                                  *
 *                                                                         *
 *   Copyright (C) 2013 by Marek Czerski                                   *
 *   ma.czerski@gmail.com                                                  *
 *                                                                         *
 *   Copyright (C) 2013 by Franck Jullien                                  *
 *   elec4fun@gmail.com                                                    *
 *                                                                         *
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

#include <jtag/jtag.h>
#include <target/register.h>
#include <target/target.h>
#include <target/breakpoints.h>
#include <target/target_type.h>
#include <helper/time_support.h>
#include <helper/fileio.h>
#include "or1k_tap.h"
#include "or1k.h"
#include "or1k_du.h"

LIST_HEAD(tap_list);
LIST_HEAD(du_list);

static int or1k_remove_breakpoint(struct target *target,
				  struct breakpoint *breakpoint);

static int or1k_read_core_reg(struct target *target, int num);
static int or1k_write_core_reg(struct target *target, int num);

static struct or1k_core_reg *or1k_core_reg_list_arch_info;

static const struct or1k_core_reg_init or1k_init_reg_list[] = {
	{"r0",        GROUP0 + 1024, "org.gnu.gdb.or1k.group0", NULL},
	{"r1",        GROUP0 + 1025, "org.gnu.gdb.or1k.group0", NULL},
	{"r2",        GROUP0 + 1026, "org.gnu.gdb.or1k.group0", NULL},
	{"r3",        GROUP0 + 1027, "org.gnu.gdb.or1k.group0", NULL},
	{"r4",        GROUP0 + 1028, "org.gnu.gdb.or1k.group0", NULL},
	{"r5",        GROUP0 + 1029, "org.gnu.gdb.or1k.group0", NULL},
	{"r6",        GROUP0 + 1030, "org.gnu.gdb.or1k.group0", NULL},
	{"r7",        GROUP0 + 1031, "org.gnu.gdb.or1k.group0", NULL},
	{"r8",        GROUP0 + 1032, "org.gnu.gdb.or1k.group0", NULL},
	{"r9",        GROUP0 + 1033, "org.gnu.gdb.or1k.group0", NULL},
	{"r10",       GROUP0 + 1034, "org.gnu.gdb.or1k.group0", NULL},
	{"r11",       GROUP0 + 1035, "org.gnu.gdb.or1k.group0", NULL},
	{"r12",       GROUP0 + 1036, "org.gnu.gdb.or1k.group0", NULL},
	{"r13",       GROUP0 + 1037, "org.gnu.gdb.or1k.group0", NULL},
	{"r14",       GROUP0 + 1038, "org.gnu.gdb.or1k.group0", NULL},
	{"r15",       GROUP0 + 1039, "org.gnu.gdb.or1k.group0", NULL},
	{"r16",       GROUP0 + 1040, "org.gnu.gdb.or1k.group0", NULL},
	{"r17",       GROUP0 + 1041, "org.gnu.gdb.or1k.group0", NULL},
	{"r18",       GROUP0 + 1042, "org.gnu.gdb.or1k.group0", NULL},
	{"r19",       GROUP0 + 1043, "org.gnu.gdb.or1k.group0", NULL},
	{"r20",       GROUP0 + 1044, "org.gnu.gdb.or1k.group0", NULL},
	{"r21",       GROUP0 + 1045, "org.gnu.gdb.or1k.group0", NULL},
	{"r22",       GROUP0 + 1046, "org.gnu.gdb.or1k.group0", NULL},
	{"r23",       GROUP0 + 1047, "org.gnu.gdb.or1k.group0", NULL},
	{"r24",       GROUP0 + 1048, "org.gnu.gdb.or1k.group0", NULL},
	{"r25",       GROUP0 + 1049, "org.gnu.gdb.or1k.group0", NULL},
	{"r26",       GROUP0 + 1050, "org.gnu.gdb.or1k.group0", NULL},
	{"r27",       GROUP0 + 1051, "org.gnu.gdb.or1k.group0", NULL},
	{"r28",       GROUP0 + 1052, "org.gnu.gdb.or1k.group0", NULL},
	{"r29",       GROUP0 + 1053, "org.gnu.gdb.or1k.group0", NULL},
	{"r30",       GROUP0 + 1054, "org.gnu.gdb.or1k.group0", NULL},
	{"r31",       GROUP0 + 1055, "org.gnu.gdb.or1k.group0", NULL},
	{"ppc",       GROUP0 + 18,   "org.gnu.gdb.or1k.group0", NULL},
	{"npc",       GROUP0 + 16,   "org.gnu.gdb.or1k.group0", NULL},
	{"sr",        GROUP0 + 17,   "org.gnu.gdb.or1k.group0", NULL},
	{"vr",        GROUP0 + 0,    "org.gnu.gdb.or1k.group0", "system"},
	{"upr",       GROUP0 + 1,    "org.gnu.gdb.or1k.group0", "system"},
	{"cpucfgr",   GROUP0 + 2,    "org.gnu.gdb.or1k.group0", "system"},
	{"dmmucfgr",  GROUP0 + 3,    "org.gnu.gdb.or1k.group0", "system"},
	{"immucfgr",  GROUP0 + 4,    "org.gnu.gdb.or1k.group0", "system"},
	{"dccfgr",    GROUP0 + 5,    "org.gnu.gdb.or1k.group0", "system"},
	{"iccfgr",    GROUP0 + 6,    "org.gnu.gdb.or1k.group0", "system"},
	{"dcfgr",     GROUP0 + 7,    "org.gnu.gdb.or1k.group0", "system"},
	{"pccfgr",    GROUP0 + 8,    "org.gnu.gdb.or1k.group0", "system"},
	{"fpcsr",     GROUP0 + 20,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr0",     GROUP0 + 32,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr1",     GROUP0 + 33,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr2",     GROUP0 + 34,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr3",     GROUP0 + 35,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr4",     GROUP0 + 36,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr5",     GROUP0 + 37,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr6",     GROUP0 + 38,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr7",     GROUP0 + 39,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr8",     GROUP0 + 40,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr9",     GROUP0 + 41,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr10",    GROUP0 + 42,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr11",    GROUP0 + 43,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr12",    GROUP0 + 44,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr13",    GROUP0 + 45,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr14",    GROUP0 + 46,   "org.gnu.gdb.or1k.group0", "system"},
	{"epcr15",    GROUP0 + 47,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear0",     GROUP0 + 48,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear1",     GROUP0 + 49,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear2",     GROUP0 + 50,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear3",     GROUP0 + 51,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear4",     GROUP0 + 52,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear5",     GROUP0 + 53,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear6",     GROUP0 + 54,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear7",     GROUP0 + 55,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear8",     GROUP0 + 56,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear9",     GROUP0 + 57,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear10",    GROUP0 + 58,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear11",    GROUP0 + 59,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear12",    GROUP0 + 60,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear13",    GROUP0 + 61,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear14",    GROUP0 + 62,   "org.gnu.gdb.or1k.group0", "system"},
	{"eear15",    GROUP0 + 63,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr0",      GROUP0 + 64,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr1",      GROUP0 + 65,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr2",      GROUP0 + 66,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr3",      GROUP0 + 67,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr4",      GROUP0 + 68,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr5",      GROUP0 + 69,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr6",      GROUP0 + 70,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr7",      GROUP0 + 71,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr8",      GROUP0 + 72,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr9",      GROUP0 + 73,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr10",     GROUP0 + 74,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr11",     GROUP0 + 75,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr12",     GROUP0 + 76,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr13",     GROUP0 + 77,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr14",     GROUP0 + 78,   "org.gnu.gdb.or1k.group0", "system"},
	{"esr15",     GROUP0 + 79,   "org.gnu.gdb.or1k.group0", "system"},

	{"dmmuucr",   GROUP1 + 0,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"dmmuupr",   GROUP1 + 1,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"dtlbeir",   GROUP1 + 2,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"datbmr0",   GROUP1 + 4,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"datbmr1",   GROUP1 + 5,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"datbmr2",   GROUP1 + 6,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"datbmr3",   GROUP1 + 7,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"datbtr0",   GROUP1 + 8,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"datbtr1",   GROUP1 + 9,    "org.gnu.gdb.or1k.group1", "dmmu"},
	{"datbtr2",   GROUP1 + 10,   "org.gnu.gdb.or1k.group1", "dmmu"},
	{"datbtr3",   GROUP1 + 11,   "org.gnu.gdb.or1k.group1", "dmmu"},

	{"immucr",    GROUP2 + 0,    "org.gnu.gdb.or1k.group2", "immu"},
	{"immupr",    GROUP2 + 1,    "org.gnu.gdb.or1k.group2", "immu"},
	{"itlbeir",   GROUP2 + 2,    "org.gnu.gdb.or1k.group2", "immu"},
	{"iatbmr0",   GROUP2 + 4,    "org.gnu.gdb.or1k.group2", "immu"},
	{"iatbmr1",   GROUP2 + 5,    "org.gnu.gdb.or1k.group2", "immu"},
	{"iatbmr2",   GROUP2 + 6,    "org.gnu.gdb.or1k.group2", "immu"},
	{"iatbmr3",   GROUP2 + 7,    "org.gnu.gdb.or1k.group2", "immu"},
	{"iatbtr0",   GROUP2 + 8,    "org.gnu.gdb.or1k.group2", "immu"},
	{"iatbtr1",   GROUP2 + 9,    "org.gnu.gdb.or1k.group2", "immu"},
	{"iatbtr2",   GROUP2 + 10,   "org.gnu.gdb.or1k.group2", "immu"},
	{"iatbtr3",   GROUP2 + 11,   "org.gnu.gdb.or1k.group2", "immu"},

	{"dccr",      GROUP3 + 0,    "org.gnu.gdb.or1k.group3", "dcache"},
	{"dcbpr",     GROUP3 + 1,    "org.gnu.gdb.or1k.group3", "dcache"},
	{"dcbfr",     GROUP3 + 2,    "org.gnu.gdb.or1k.group3", "dcache"},
	{"dcbir",     GROUP3 + 3,    "org.gnu.gdb.or1k.group3", "dcache"},
	{"dcbwr",     GROUP3 + 4,    "org.gnu.gdb.or1k.group3", "dcache"},
	{"dcblr",     GROUP3 + 5,    "org.gnu.gdb.or1k.group3", "dcache"},

	{"iccr",      GROUP4 + 0,    "org.gnu.gdb.or1k.group4", "icache"},
	{"icbpr",     GROUP4 + 1,    "org.gnu.gdb.or1k.group4", "icache"},
	{"icbir",     GROUP4 + 2,    "org.gnu.gdb.or1k.group4", "icache"},
	{"icblr",     GROUP4 + 3,    "org.gnu.gdb.or1k.group4", "icache"},

	{"maclo",     GROUP5 + 0,    "org.gnu.gdb.or1k.group5", "mac"},
	{"machi",     GROUP5 + 1,    "org.gnu.gdb.or1k.group5", "mac"},

	{"dvr0",      GROUP6 + 0,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dvr1",      GROUP6 + 1,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dvr2",      GROUP6 + 2,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dvr3",      GROUP6 + 3,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dvr4",      GROUP6 + 4,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dvr5",      GROUP6 + 5,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dvr6",      GROUP6 + 6,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dvr7",      GROUP6 + 7,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dcr0",      GROUP6 + 8,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dcr1",      GROUP6 + 9,    "org.gnu.gdb.or1k.group6", "debug"},
	{"dcr2",      GROUP6 + 10,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dcr3",      GROUP6 + 11,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dcr4",      GROUP6 + 12,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dcr5",      GROUP6 + 13,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dcr6",      GROUP6 + 14,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dcr7",      GROUP6 + 15,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dmr1",      GROUP6 + 16,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dmr2",      GROUP6 + 17,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dcwr0",     GROUP6 + 18,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dcwr1",     GROUP6 + 19,   "org.gnu.gdb.or1k.group6", "debug"},
	{"dsr",       GROUP6 + 20,   "org.gnu.gdb.or1k.group6", "debug"},
	{"drr",       GROUP6 + 21,   "org.gnu.gdb.or1k.group6", "debug"},

	{"pccr0",     GROUP7 + 0,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pccr1",     GROUP7 + 1,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pccr2",     GROUP7 + 2,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pccr3",     GROUP7 + 3,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pccr4",     GROUP7 + 4,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pccr5",     GROUP7 + 5,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pccr6",     GROUP7 + 6,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pccr7",     GROUP7 + 7,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pcmr0",     GROUP7 + 8,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pcmr1",     GROUP7 + 9,    "org.gnu.gdb.or1k.group7", "perf"},
	{"pcmr2",     GROUP7 + 10,   "org.gnu.gdb.or1k.group7", "perf"},
	{"pcmr3",     GROUP7 + 11,   "org.gnu.gdb.or1k.group7", "perf"},
	{"pcmr4",     GROUP7 + 12,   "org.gnu.gdb.or1k.group7", "perf"},
	{"pcmr5",     GROUP7 + 13,   "org.gnu.gdb.or1k.group7", "perf"},
	{"pcmr6",     GROUP7 + 14,   "org.gnu.gdb.or1k.group7", "perf"},
	{"pcmr7",     GROUP7 + 15,   "org.gnu.gdb.or1k.group7", "perf"},

	{"pmr",       GROUP8 + 0,    "org.gnu.gdb.or1k.group8", "power"},

	{"picmr",     GROUP9 + 0,    "org.gnu.gdb.or1k.group9", "pic"},
	{"picsr",     GROUP9 + 2,    "org.gnu.gdb.or1k.group9", "pic"},

	{"ttmr",      GROUP10 + 0,   "org.gnu.gdb.or1k.group10", "timer"},
	{"ttcr",      GROUP10 + 1,   "org.gnu.gdb.or1k.group10", "timer"},
};

static int or1k_add_reg(struct target *target, struct or1k_core_reg *new_reg)
{
	struct or1k_common *or1k = target_to_or1k(target);
	int reg_list_size = or1k->nb_regs * sizeof(struct or1k_core_reg);

	or1k_core_reg_list_arch_info = realloc(or1k_core_reg_list_arch_info,
				reg_list_size + sizeof(struct or1k_core_reg));

	memcpy(&or1k_core_reg_list_arch_info[or1k->nb_regs], new_reg,
		sizeof(struct or1k_core_reg));

	or1k_core_reg_list_arch_info[or1k->nb_regs].list_num = or1k->nb_regs;

	or1k->nb_regs++;

	return ERROR_OK;
}

static int or1k_create_reg_list(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);

	LOG_DEBUG("-");

	or1k_core_reg_list_arch_info = malloc(ARRAY_SIZE(or1k_init_reg_list) *
				       sizeof(struct or1k_core_reg));

	for (int i = 0; i < (int)ARRAY_SIZE(or1k_init_reg_list); i++) {
		or1k_core_reg_list_arch_info[i].name = or1k_init_reg_list[i].name;
		or1k_core_reg_list_arch_info[i].spr_num = or1k_init_reg_list[i].spr_num;
		or1k_core_reg_list_arch_info[i].group = or1k_init_reg_list[i].group;
		or1k_core_reg_list_arch_info[i].feature = or1k_init_reg_list[i].feature;
		or1k_core_reg_list_arch_info[i].list_num = i;
		or1k_core_reg_list_arch_info[i].target = NULL;
		or1k_core_reg_list_arch_info[i].or1k_common = NULL;
	}

	or1k->nb_regs = ARRAY_SIZE(or1k_init_reg_list);

	struct or1k_core_reg new_reg;
	new_reg.target = NULL;
	new_reg.or1k_common = NULL;

	char name[32];
	for (int way = 0; way < 4; way++) {
		for (int i = 0; i < 128; i++) {

			sprintf(name, "dtlbw%dmr%d", way, i);
			new_reg.name = strdup(name);
			new_reg.spr_num = GROUP1 + 512 + i + (way * 256);
			new_reg.feature = "org.gnu.gdb.or1k.group1";
			new_reg.group = "dmmu";
			or1k_add_reg(target, &new_reg);

			sprintf(name, "dtlbw%dtr%d", way, i);
			new_reg.name = strdup(name);
			new_reg.spr_num = GROUP1 + 640 + i + (way * 256);
			new_reg.feature = "org.gnu.gdb.or1k.group1";
			new_reg.group = "dmmu";
			or1k_add_reg(target, &new_reg);


			sprintf(name, "itlbw%dmr%d", way, i);
			new_reg.name = strdup(name);
			new_reg.spr_num = GROUP2 + 512 + i + (way * 256);
			new_reg.feature = "org.gnu.gdb.or1k.group2";
			new_reg.group = "immu";
			or1k_add_reg(target, &new_reg);


			sprintf(name, "itlbw%dtr%d", way, i);
			new_reg.name = strdup(name);
			new_reg.spr_num = GROUP2 + 640 + i + (way * 256);
			new_reg.feature = "org.gnu.gdb.or1k.group2";
			new_reg.group = "immu";
			or1k_add_reg(target, &new_reg);

		}
	}

	return ERROR_OK;
}

static int or1k_jtag_read_regs(struct or1k_common *or1k, uint32_t *regs)
{
	struct or1k_du *du_core = or1k_jtag_to_du(&or1k->jtag);

	LOG_DEBUG("-");

	return du_core->or1k_jtag_read_cpu(&or1k->jtag,
			or1k->arch_info[OR1K_REG_R0].spr_num, OR1K_REG_R31 + 1,
			regs + OR1K_REG_R0);
}

static int or1k_jtag_write_regs(struct or1k_common *or1k, uint32_t *regs)
{
	struct or1k_du *du_core = or1k_jtag_to_du(&or1k->jtag);

	LOG_DEBUG("-");

	return du_core->or1k_jtag_write_cpu(&or1k->jtag,
			or1k->arch_info[OR1K_REG_R0].spr_num, OR1K_REG_R31 + 1,
			&regs[OR1K_REG_R0]);
}

static int or1k_save_context(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	int regs_read = 0;
	int retval;

	LOG_DEBUG("-");

	for (int i = 0; i < OR1KNUMCOREREGS; i++) {
		if (!or1k->core_cache->reg_list[i].valid) {
			if (i == OR1K_REG_PPC || i == OR1K_REG_NPC || i == OR1K_REG_SR) {
				retval = du_core->or1k_jtag_read_cpu(&or1k->jtag,
						or1k->arch_info[i].spr_num, 1,
						&or1k->core_regs[i]);
				if (retval != ERROR_OK)
					return retval;
			} else if (!regs_read) {
				/* read gpr registers at once (but only one time in this loop) */
				retval = or1k_jtag_read_regs(or1k, or1k->core_regs);
				if (retval != ERROR_OK)
					return retval;
				/* prevent next reads in this loop */
				regs_read = 1;
			}
			/* We've just updated the core_reg[i], now update
			   the core cache */
			or1k_read_core_reg(target, i);
		}
	}

	return ERROR_OK;
}

static int or1k_restore_context(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	int reg_write = 0;
	int retval;

	LOG_DEBUG("-");

	for (int i = 0; i < OR1KNUMCOREREGS; i++) {
		if (or1k->core_cache->reg_list[i].dirty) {
			or1k_write_core_reg(target, i);

			if (i == OR1K_REG_PPC || i == OR1K_REG_NPC || i == OR1K_REG_SR) {
				retval = du_core->or1k_jtag_write_cpu(&or1k->jtag,
						or1k->arch_info[i].spr_num, 1,
						&or1k->core_regs[i]);
				if (retval != ERROR_OK) {
					LOG_ERROR("Error while restoring context");
					return retval;
				}
			} else
				reg_write = 1;
		}
	}

	if (reg_write) {
		/* read gpr registers at once (but only one time in this loop) */
		retval = or1k_jtag_write_regs(or1k, or1k->core_regs);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error while restoring context");
			return retval;
		}
	}

	return ERROR_OK;
}

static int or1k_read_core_reg(struct target *target, int num)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	uint32_t reg_value;

	LOG_DEBUG("-");

	if ((num < 0) || (num >= or1k->nb_regs))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if ((num >= 0) && (num < OR1KNUMCOREREGS)) {
		reg_value = or1k->core_regs[num];
		buf_set_u32(or1k->core_cache->reg_list[num].value, 0, 32, reg_value);
		LOG_DEBUG("Read core reg %i value 0x%08" PRIx32, num, reg_value);
		or1k->core_cache->reg_list[num].valid = true;
		or1k->core_cache->reg_list[num].dirty = false;
	} else {
		/* This is an spr, always read value from HW */
		int retval = du_core->or1k_jtag_read_cpu(&or1k->jtag,
							 or1k->arch_info[num].spr_num, 1, &reg_value);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error while reading spr 0x%08" PRIx32, or1k->arch_info[num].spr_num);
			return retval;
		}
		buf_set_u32(or1k->core_cache->reg_list[num].value, 0, 32, reg_value);
		LOG_DEBUG("Read spr reg %i value 0x%08" PRIx32, num, reg_value);
	}

	return ERROR_OK;
}

static int or1k_write_core_reg(struct target *target, int num)
{
	struct or1k_common *or1k = target_to_or1k(target);

	LOG_DEBUG("-");

	if ((num < 0) || (num >= OR1KNUMCOREREGS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t reg_value = buf_get_u32(or1k->core_cache->reg_list[num].value, 0, 32);
	or1k->core_regs[num] = reg_value;
	LOG_DEBUG("Write core reg %i value 0x%08" PRIx32, num, reg_value);
	or1k->core_cache->reg_list[num].valid = true;
	or1k->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

static int or1k_get_core_reg(struct reg *reg)
{
	struct or1k_core_reg *or1k_reg = reg->arch_info;
	struct target *target = or1k_reg->target;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	return or1k_read_core_reg(target, or1k_reg->list_num);
}

static int or1k_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct or1k_core_reg *or1k_reg = reg->arch_info;
	struct target *target = or1k_reg->target;
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	uint32_t value = buf_get_u32(buf, 0, 32);

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (or1k_reg->list_num < OR1KNUMCOREREGS) {
		buf_set_u32(reg->value, 0, 32, value);
		reg->dirty = true;
		reg->valid = true;
	} else {
		/* This is an spr, write it to the HW */
		int retval = du_core->or1k_jtag_write_cpu(&or1k->jtag,
							  or1k_reg->spr_num, 1, &value);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error while writing spr 0x%08" PRIx32, or1k_reg->spr_num);
			return retval;
		}
	}

	return ERROR_OK;
}

static const struct reg_arch_type or1k_reg_type = {
	.get = or1k_get_core_reg,
	.set = or1k_set_core_reg,
};

static struct reg_cache *or1k_build_reg_cache(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(or1k->nb_regs, sizeof(struct reg));
	struct or1k_core_reg *arch_info =
		malloc((or1k->nb_regs) * sizeof(struct or1k_core_reg));
	struct reg_feature *feature;

	LOG_DEBUG("-");

	/* Build the process context cache */
	cache->name = "OpenRISC 1000 registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = or1k->nb_regs;
	(*cache_p) = cache;
	or1k->core_cache = cache;
	or1k->arch_info = arch_info;

	for (int i = 0; i < or1k->nb_regs; i++) {
		arch_info[i] = or1k_core_reg_list_arch_info[i];
		arch_info[i].target = target;
		arch_info[i].or1k_common = or1k;
		reg_list[i].name = or1k_core_reg_list_arch_info[i].name;

		feature = malloc(sizeof(struct reg_feature));
		feature->name = or1k_core_reg_list_arch_info[i].feature;
		reg_list[i].feature = feature;

		reg_list[i].group = or1k_core_reg_list_arch_info[i].group;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].type = &or1k_reg_type;
		reg_list[i].arch_info = &arch_info[i];
		reg_list[i].number = i;
		reg_list[i].exist = true;
	}

	return cache;
}

static int or1k_debug_entry(struct target *target)
{
	LOG_DEBUG("-");

	int retval = or1k_save_context(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling or1k_save_context");
		return retval;
	}

	struct or1k_common *or1k = target_to_or1k(target);
	uint32_t addr = or1k->core_regs[OR1K_REG_NPC];

	if (breakpoint_find(target, addr))
		/* Halted on a breakpoint, step back to permit executing the instruction there */
		retval = or1k_set_core_reg(&or1k->core_cache->reg_list[OR1K_REG_NPC],
					   (uint8_t *)&addr);

	return retval;
}

static int or1k_halt(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);

	LOG_DEBUG("target->state: %s",
		  target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("Target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("Target was in unknown state when halt was requested");

	if (target->state == TARGET_RESET) {
		if ((jtag_get_reset_config() & RESET_SRST_PULLS_TRST) &&
		    jtag_get_srst()) {
			LOG_ERROR("Can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		} else {
			target->debug_reason = DBG_REASON_DBGRQ;
			return ERROR_OK;
		}
	}

	int retval = du_core->or1k_cpu_stall(&or1k->jtag, CPU_STALL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Impossible to stall the CPU");
		return retval;
	}

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int or1k_is_cpu_running(struct target *target, int *running)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	int retval;
	int tries = 0;
	const int RETRIES_MAX = 5;

	/* Have a retry loop to determine of the CPU is running.
	   If target has been hard reset for any reason, it might take a couple
	   of goes before it's ready again.
	*/
	while (tries < RETRIES_MAX) {

		tries++;

		retval = du_core->or1k_is_cpu_running(&or1k->jtag, running);
		if (retval != ERROR_OK) {
			LOG_WARNING("Debug IF CPU control reg read failure.");
			/* Try once to restart the JTAG infrastructure -
			   quite possibly the board has just been reset. */
			LOG_WARNING("Resetting JTAG TAP state and reconnecting to debug IF.");
			du_core->or1k_jtag_init(&or1k->jtag);

			LOG_WARNING("...attempt %d of %d", tries, RETRIES_MAX);

			alive_sleep(2);

			continue;
		} else
			return ERROR_OK;
	}

	LOG_ERROR("Could not re-establish communication with target");
	return retval;
}

static int or1k_poll(struct target *target)
{
	int retval;
	int running;

	retval = or1k_is_cpu_running(target, &running);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling or1k_is_cpu_running");
		return retval;
	}

	/* check for processor halted */
	if (!running) {
		/* It's actually stalled, so update our software's state */
		if ((target->state == TARGET_RUNNING) ||
		    (target->state == TARGET_RESET)) {

			target->state = TARGET_HALTED;

			retval = or1k_debug_entry(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error while calling or1k_debug_entry");
				return retval;
			}

			target_call_event_callbacks(target,
						    TARGET_EVENT_HALTED);
		} else if (target->state == TARGET_DEBUG_RUNNING) {
			target->state = TARGET_HALTED;

			retval = or1k_debug_entry(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error while calling or1k_debug_entry");
				return retval;
			}

			target_call_event_callbacks(target,
						    TARGET_EVENT_DEBUG_HALTED);
		}
	} else { /* ... target is running */

		/* If target was supposed to be stalled, stall it again */
		if  (target->state == TARGET_HALTED) {

			target->state = TARGET_RUNNING;

			retval = or1k_halt(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error while calling or1k_halt");
				return retval;
			}

			retval = or1k_debug_entry(target);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error while calling or1k_debug_entry");
				return retval;
			}

			target_call_event_callbacks(target,
						    TARGET_EVENT_DEBUG_HALTED);
		}

		target->state = TARGET_RUNNING;

	}

	return ERROR_OK;
}

static int or1k_assert_reset(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);

	LOG_DEBUG("-");

	int retval = du_core->or1k_cpu_reset(&or1k->jtag, CPU_RESET);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while asserting RESET");
		return retval;
	}

	return ERROR_OK;
}

static int or1k_deassert_reset(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);

	LOG_DEBUG("-");

	int retval = du_core->or1k_cpu_reset(&or1k->jtag, CPU_NOT_RESET);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while deasserting RESET");
		return retval;
	}

	return ERROR_OK;
}

static int or1k_soft_reset_halt(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);

	LOG_DEBUG("-");

	int retval = du_core->or1k_cpu_stall(&or1k->jtag, CPU_STALL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while stalling the CPU");
		return retval;
	}

	retval = or1k_assert_reset(target);
	if (retval != ERROR_OK)
		return retval;

	retval = or1k_deassert_reset(target);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static bool is_any_soft_breakpoint(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	LOG_DEBUG("-");

	while (breakpoint)
		if (breakpoint->type == BKPT_SOFT)
			return true;

	return false;
}

static int or1k_resume_or_step(struct target *target, int current,
			       uint32_t address, int handle_breakpoints,
			       int debug_execution, int step)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;
	uint32_t debug_reg_list[OR1K_DEBUG_REG_NUM];

	LOG_DEBUG("Addr: 0x%" PRIx32 ", stepping: %s, handle breakpoints %s\n",
		  address, step ? "yes" : "no", handle_breakpoints ? "yes" : "no");

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution)
		target_free_all_working_areas(target);

	/* current ? continue on current pc : continue at <address> */
	if (!current)
		buf_set_u32(or1k->core_cache->reg_list[OR1K_REG_NPC].value, 0,
			    32, address);

	int retval = or1k_restore_context(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while calling or1k_restore_context");
		return retval;
	}

	/* read debug registers (starting from DMR1 register) */
	retval = du_core->or1k_jtag_read_cpu(&or1k->jtag, OR1K_DMR1_CPU_REG_ADD,
					     OR1K_DEBUG_REG_NUM, debug_reg_list);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while reading debug registers");
		return retval;
	}

	/* Clear Debug Reason Register (DRR) */
	debug_reg_list[OR1K_DEBUG_REG_DRR] = 0;

	/* Clear watchpoint break generation in Debug Mode Register 2 (DMR2) */
	debug_reg_list[OR1K_DEBUG_REG_DMR2] &= ~OR1K_DMR2_WGB;
	if (step)
		/* Set the single step trigger in Debug Mode Register 1 (DMR1) */
		debug_reg_list[OR1K_DEBUG_REG_DMR1] |= OR1K_DMR1_ST | OR1K_DMR1_BT;
	else
		/* Clear the single step trigger in Debug Mode Register 1 (DMR1) */
		debug_reg_list[OR1K_DEBUG_REG_DMR1] &= ~(OR1K_DMR1_ST | OR1K_DMR1_BT);

	/* Set traps to be handled by the debug unit in the Debug Stop
	   Register (DSR). Check if we have any software breakpoints in
	   place before setting this value - the kernel, for instance,
	   relies on l.trap instructions not stalling the processor ! */
	if (is_any_soft_breakpoint(target) == true)
		debug_reg_list[OR1K_DEBUG_REG_DSR] |= OR1K_DSR_TE;

	/* Write debug registers (starting from DMR1 register) */
	retval = du_core->or1k_jtag_write_cpu(&or1k->jtag, OR1K_DMR1_CPU_REG_ADD,
					      OR1K_DEBUG_REG_NUM, debug_reg_list);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while writing back debug registers");
		return retval;
	}

	resume_pc = buf_get_u32(or1k->core_cache->reg_list[OR1K_REG_NPC].value,
				0, 32);

	/* The front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_DEBUG("Unset breakpoint at 0x%08" TARGET_PRIxADDR, breakpoint->address);
			retval = or1k_remove_breakpoint(target, breakpoint);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* Unstall time */
	retval = du_core->or1k_cpu_stall(&or1k->jtag, CPU_UNSTALL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while unstalling the CPU");
		return retval;
	}

	if (step)
		target->debug_reason = DBG_REASON_SINGLESTEP;
	else
		target->debug_reason = DBG_REASON_NOTHALTED;

	/* Registers are now invalid */
	register_cache_invalidate(or1k->core_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("Target resumed at 0x%08" PRIx32, resume_pc);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("Target debug resumed at 0x%08" PRIx32, resume_pc);
	}

	return ERROR_OK;
}

static int or1k_resume(struct target *target, int current,
		       target_addr_t address, int handle_breakpoints,
		       int debug_execution)
{
	return or1k_resume_or_step(target, current, address,
				   handle_breakpoints,
				   debug_execution,
				   NO_SINGLE_STEP);
}

static int or1k_step(struct target *target, int current,
		     target_addr_t address, int handle_breakpoints)
{
	return or1k_resume_or_step(target, current, address,
				   handle_breakpoints,
				   0,
				   SINGLE_STEP);

}

static int or1k_add_breakpoint(struct target *target,
			       struct breakpoint *breakpoint)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	uint8_t data;

	LOG_DEBUG("Adding breakpoint: addr 0x%08" TARGET_PRIxADDR ", len %d, type %d, set: %d, id: %" PRIu32,
		  breakpoint->address, breakpoint->length, breakpoint->type,
		  breakpoint->set, breakpoint->unique_id);

	/* Only support SW breakpoints for now. */
	if (breakpoint->type == BKPT_HARD)
		LOG_ERROR("HW breakpoints not supported for now. Doing SW breakpoint.");

	/* Read and save the instruction */
	int retval = du_core->or1k_jtag_read_memory(&or1k->jtag,
					 breakpoint->address,
					 4,
					 1,
					 &data);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while reading the instruction at 0x%08" TARGET_PRIxADDR,
			   breakpoint->address);
		return retval;
	}

	free(breakpoint->orig_instr);

	breakpoint->orig_instr = malloc(breakpoint->length);
	memcpy(breakpoint->orig_instr, &data, breakpoint->length);

	/* Sub in the OR1K trap instruction */
	uint8_t or1k_trap_insn[4];
	target_buffer_set_u32(target, or1k_trap_insn, OR1K_TRAP_INSTR);
	retval = du_core->or1k_jtag_write_memory(&or1k->jtag,
					  breakpoint->address,
					  4,
					  1,
					  or1k_trap_insn);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while writing OR1K_TRAP_INSTR at 0x%08" TARGET_PRIxADDR,
			   breakpoint->address);
		return retval;
	}

	/* invalidate instruction cache */
	uint32_t addr = breakpoint->address;
	retval = du_core->or1k_jtag_write_cpu(&or1k->jtag,
			OR1K_ICBIR_CPU_REG_ADD, 1, &addr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while invalidating the ICACHE");
		return retval;
	}

	return ERROR_OK;
}

static int or1k_remove_breakpoint(struct target *target,
				  struct breakpoint *breakpoint)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);

	LOG_DEBUG("Removing breakpoint: addr 0x%08" TARGET_PRIxADDR ", len %d, type %d, set: %d, id: %" PRIu32,
		  breakpoint->address, breakpoint->length, breakpoint->type,
		  breakpoint->set, breakpoint->unique_id);

	/* Only support SW breakpoints for now. */
	if (breakpoint->type == BKPT_HARD)
		LOG_ERROR("HW breakpoints not supported for now. Doing SW breakpoint.");

	/* Replace the removed instruction */
	int retval = du_core->or1k_jtag_write_memory(&or1k->jtag,
					  breakpoint->address,
					  4,
					  1,
					  breakpoint->orig_instr);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while writing back the instruction at 0x%08" TARGET_PRIxADDR,
			   breakpoint->address);
		return retval;
	}

	/* invalidate instruction cache */
	uint32_t addr = breakpoint->address;
	retval = du_core->or1k_jtag_write_cpu(&or1k->jtag,
			OR1K_ICBIR_CPU_REG_ADD, 1, &addr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error while invalidating the ICACHE");
		return retval;
	}

	return ERROR_OK;
}

static int or1k_add_watchpoint(struct target *target,
			       struct watchpoint *watchpoint)
{
	LOG_ERROR("%s: implement me", __func__);
	return ERROR_OK;
}

static int or1k_remove_watchpoint(struct target *target,
				  struct watchpoint *watchpoint)
{
	LOG_ERROR("%s: implement me", __func__);
	return ERROR_OK;
}

static int or1k_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);

	LOG_DEBUG("Read memory at 0x%08" TARGET_PRIxADDR ", size: %" PRIu32 ", count: 0x%08" PRIx32, address, size, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !buffer) {
		LOG_ERROR("Bad arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u))) {
		LOG_ERROR("Can't handle unaligned memory access");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	return du_core->or1k_jtag_read_memory(&or1k->jtag, address, size, count, buffer);
}

static int or1k_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);

	LOG_DEBUG("Write memory at 0x%08" TARGET_PRIxADDR ", size: %" PRIu32 ", count: 0x%08" PRIx32, address, size, count);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !buffer) {
		LOG_ERROR("Bad arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u))) {
		LOG_ERROR("Can't handle unaligned memory access");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	return du_core->or1k_jtag_write_memory(&or1k->jtag, address, size, count, buffer);
}

static int or1k_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	struct or1k_jtag *jtag = &or1k->jtag;

	if (du_core == NULL) {
		LOG_ERROR("No debug unit selected");
		return ERROR_FAIL;
	}

	if (jtag->tap_ip == NULL) {
		LOG_ERROR("No tap selected");
		return ERROR_FAIL;
	}

	or1k->jtag.tap = target->tap;
	or1k->jtag.or1k_jtag_inited = 0;
	or1k->jtag.or1k_jtag_module_selected = -1;
	or1k->jtag.target = target;

	or1k_build_reg_cache(target);

	return ERROR_OK;
}

static int or1k_target_create(struct target *target, Jim_Interp *interp)
{
	if (target->tap == NULL)
		return ERROR_FAIL;

	struct or1k_common *or1k = calloc(1, sizeof(struct or1k_common));

	target->arch_info = or1k;

	or1k_create_reg_list(target);

	or1k_tap_vjtag_register();
	or1k_tap_xilinx_bscan_register();
	or1k_tap_mohor_register();

	or1k_du_adv_register();

	return ERROR_OK;
}

static int or1k_examine(struct target *target)
{
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);

	if (!target_was_examined(target)) {

		target_set_examined(target);

		int running;

		int retval = du_core->or1k_is_cpu_running(&or1k->jtag, &running);
		if (retval != ERROR_OK) {
			LOG_ERROR("Couldn't read the CPU state");
			return retval;
		} else {
			if (running)
				target->state = TARGET_RUNNING;
			else {
				LOG_DEBUG("Target is halted");

				/* This is the first time we examine the target,
				 * it is stalled and we don't know why. Let's
				 * assume this is because of a debug reason.
				 */
				if (target->state == TARGET_UNKNOWN)
					target->debug_reason = DBG_REASON_DBGRQ;

				target->state = TARGET_HALTED;
			}
		}
	}

	return ERROR_OK;
}

static int or1k_arch_state(struct target *target)
{
	return ERROR_OK;
}

static int or1k_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
			  int *reg_list_size, enum target_register_class reg_class)
{
	struct or1k_common *or1k = target_to_or1k(target);

	if (reg_class == REG_CLASS_GENERAL) {
		/* We will have this called whenever GDB connects. */
		int retval = or1k_save_context(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error while calling or1k_save_context");
			return retval;
		}
		*reg_list_size = OR1KNUMCOREREGS;
		/* this is free()'d back in gdb_server.c's gdb_get_register_packet() */
		*reg_list = malloc((*reg_list_size) * sizeof(struct reg *));

		for (int i = 0; i < OR1KNUMCOREREGS; i++)
			(*reg_list)[i] = &or1k->core_cache->reg_list[i];
	} else {
		*reg_list_size = or1k->nb_regs;
		*reg_list = malloc((*reg_list_size) * sizeof(struct reg *));

		for (int i = 0; i < or1k->nb_regs; i++)
			(*reg_list)[i] = &or1k->core_cache->reg_list[i];
	}

	return ERROR_OK;

}

static int or1k_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info)
{
	return ERROR_FAIL;
}

static int or1k_checksum_memory(struct target *target, target_addr_t address,
		uint32_t count, uint32_t *checksum)
{
	return ERROR_FAIL;
}

static int or1k_profiling(struct target *target, uint32_t *samples,
		uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds)
{
	struct timeval timeout, now;
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_du *du_core = or1k_to_du(or1k);
	int retval = ERROR_OK;

	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, seconds, 0);

	LOG_INFO("Starting or1k profiling. Sampling npc as fast as we can...");

	/* Make sure the target is running */
	target_poll(target);
	if (target->state == TARGET_HALTED)
		retval = target_resume(target, 1, 0, 0, 0);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error while resuming target");
		return retval;
	}

	uint32_t sample_count = 0;

	for (;;) {
		uint32_t reg_value;
		retval = du_core->or1k_jtag_read_cpu(&or1k->jtag, GROUP0 + 16 /* NPC */, 1, &reg_value);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error while reading NPC");
			return retval;
		}

		samples[sample_count++] = reg_value;

		gettimeofday(&now, NULL);
		if ((sample_count >= max_num_samples) || timeval_compare(&now, &timeout) > 0) {
			LOG_INFO("Profiling completed. %" PRIu32 " samples.", sample_count);
			break;
		}
	}

	*num_samples = sample_count;
	return retval;
}

COMMAND_HANDLER(or1k_tap_select_command_handler)
{
	struct target *target = get_current_target(CMD_CTX);
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_jtag *jtag = &or1k->jtag;
	struct or1k_tap_ip *or1k_tap;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(or1k_tap, &tap_list, list) {
		if (or1k_tap->name) {
			if (!strcmp(CMD_ARGV[0], or1k_tap->name)) {
				jtag->tap_ip = or1k_tap;
				LOG_INFO("%s tap selected", or1k_tap->name);
				return ERROR_OK;
			}
		}
	}

	LOG_ERROR("%s unknown, no tap selected", CMD_ARGV[0]);
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(or1k_tap_list_command_handler)
{
	struct or1k_tap_ip *or1k_tap;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(or1k_tap, &tap_list, list) {
		if (or1k_tap->name)
			command_print(CMD, "%s", or1k_tap->name);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(or1k_du_select_command_handler)
{
	struct target *target = get_current_target(CMD_CTX);
	struct or1k_common *or1k = target_to_or1k(target);
	struct or1k_jtag *jtag = &or1k->jtag;
	struct or1k_du *or1k_du;

	if (CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(or1k_du, &du_list, list) {
		if (or1k_du->name) {
			if (!strcmp(CMD_ARGV[0], or1k_du->name)) {
				jtag->du_core = or1k_du;
				LOG_INFO("%s debug unit selected", or1k_du->name);

				if (CMD_ARGC == 2) {
					int options;
					COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], options);
					or1k_du->options = options;
					LOG_INFO("Option %x is passed to %s debug unit"
						 , options, or1k_du->name);
				}

				return ERROR_OK;
			}
		}
	}

	LOG_ERROR("%s unknown, no debug unit selected", CMD_ARGV[0]);
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(or1k_du_list_command_handler)
{
	struct or1k_du *or1k_du;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(or1k_du, &du_list, list) {
		if (or1k_du->name)
			command_print(CMD, "%s", or1k_du->name);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(or1k_addreg_command_handler)
{
	struct target *target = get_current_target(CMD_CTX);
	struct or1k_core_reg new_reg;

	if (CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	new_reg.target = NULL;
	new_reg.or1k_common = NULL;

	uint32_t addr;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], addr);

	new_reg.name = strdup(CMD_ARGV[0]);
	new_reg.spr_num = addr;
	new_reg.feature = strdup(CMD_ARGV[2]);
	new_reg.group = strdup(CMD_ARGV[3]);

	or1k_add_reg(target, &new_reg);

	LOG_DEBUG("Add reg \"%s\" @ 0x%08" PRIx32 ", group \"%s\", feature \"%s\"",
		  new_reg.name, addr, new_reg.group, new_reg.feature);

	return ERROR_OK;
}

static const struct command_registration or1k_hw_ip_command_handlers[] = {
	{
		.name = "tap_select",
		.handler = or1k_tap_select_command_handler,
		.mode = COMMAND_ANY,
		.usage = "name",
		.help = "Select the TAP core to use",
	},
	{
		.name = "tap_list",
		.handler = or1k_tap_list_command_handler,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Display available TAP core",
	},
	{
		.name = "du_select",
		.handler = or1k_du_select_command_handler,
		.mode = COMMAND_ANY,
		.usage = "name",
		.help = "Select the Debug Unit core to use",
	},
	{
		.name = "du_list",
		.handler = or1k_du_list_command_handler,
		.mode = COMMAND_ANY,
		.usage = "select_tap name",
		.help = "Display available Debug Unit core",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration or1k_reg_command_handlers[] = {
	{
		.name = "addreg",
		.handler = or1k_addreg_command_handler,
		.mode = COMMAND_ANY,
		.usage = "name addr feature group",
		.help = "Add a register to the register list",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration or1k_command_handlers[] = {
	{
		.chain = or1k_reg_command_handlers,
	},
	{
		.chain = or1k_hw_ip_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


struct target_type or1k_target = {
	.name = "or1k",

	.poll = or1k_poll,
	.arch_state = or1k_arch_state,

	.target_request_data = NULL,

	.halt = or1k_halt,
	.resume = or1k_resume,
	.step = or1k_step,

	.assert_reset = or1k_assert_reset,
	.deassert_reset = or1k_deassert_reset,
	.soft_reset_halt = or1k_soft_reset_halt,

	.get_gdb_reg_list = or1k_get_gdb_reg_list,

	.read_memory = or1k_read_memory,
	.write_memory = or1k_write_memory,
	.checksum_memory = or1k_checksum_memory,

	.commands = or1k_command_handlers,
	.add_breakpoint = or1k_add_breakpoint,
	.remove_breakpoint = or1k_remove_breakpoint,
	.add_watchpoint = or1k_add_watchpoint,
	.remove_watchpoint = or1k_remove_watchpoint,

	.target_create = or1k_target_create,
	.init_target = or1k_init_target,
	.examine = or1k_examine,

	.get_gdb_fileio_info = or1k_get_gdb_fileio_info,

	.profiling = or1k_profiling,
};
