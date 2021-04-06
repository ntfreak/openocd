/***************************************************************************
 *   Copyright (C) 2017 by James Murray <james@nscc.info                   *
 *   Based on code:                                                        *
 *       Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>   *
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

#ifndef MPC5XXX
#define MPC5XXX

struct target;

#define	MPC5XXX_NUMCOREREGS	72 /* GDB/Zylin CDT asks for high non-existent regs */

enum mpc5xxx_reg_nums {
	MPC5XXX_REG_R0 = 0,
	MPC5XXX_REG_R1, /* also SP ? */
	MPC5XXX_REG_R2,
	MPC5XXX_REG_R3,
	MPC5XXX_REG_R4,
	MPC5XXX_REG_R5,
	MPC5XXX_REG_R6,
	MPC5XXX_REG_R7,
	MPC5XXX_REG_R8,
	MPC5XXX_REG_R9,
	MPC5XXX_REG_R10,
	MPC5XXX_REG_R11,
	MPC5XXX_REG_R12,
	MPC5XXX_REG_R13,
	MPC5XXX_REG_R14,
	MPC5XXX_REG_R15,
	MPC5XXX_REG_R16,
	MPC5XXX_REG_R17,
	MPC5XXX_REG_R18,
	MPC5XXX_REG_R19,
	MPC5XXX_REG_R20,
	MPC5XXX_REG_R21,
	MPC5XXX_REG_R22,
	MPC5XXX_REG_R23,
	MPC5XXX_REG_R24,
	MPC5XXX_REG_R25,
	MPC5XXX_REG_R26,
	MPC5XXX_REG_R27,
	MPC5XXX_REG_R28,
	MPC5XXX_REG_R29,
	MPC5XXX_REG_R30,
	MPC5XXX_REG_R31,
	MPC5XXX_REG_R32,
	MPC5XXX_REG_R33,
	MPC5XXX_REG_R34,
	MPC5XXX_REG_R35,
	MPC5XXX_REG_R36,
	MPC5XXX_REG_R37,
	MPC5XXX_REG_R38,
	MPC5XXX_REG_R39,
	MPC5XXX_REG_R40,
	MPC5XXX_REG_R41,
	MPC5XXX_REG_R42,
	MPC5XXX_REG_R43,
	MPC5XXX_REG_R44,
	MPC5XXX_REG_R45,
	MPC5XXX_REG_R46,
	MPC5XXX_REG_R47,
	MPC5XXX_REG_R48,
	MPC5XXX_REG_R49,
	MPC5XXX_REG_R50,
	MPC5XXX_REG_R51,
	MPC5XXX_REG_R52,
	MPC5XXX_REG_R53,
	MPC5XXX_REG_R54,
	MPC5XXX_REG_R55,
	MPC5XXX_REG_R56,
	MPC5XXX_REG_R57,
	MPC5XXX_REG_R58,
	MPC5XXX_REG_R59,
	MPC5XXX_REG_R60,
	MPC5XXX_REG_R61,
	MPC5XXX_REG_R62,
	MPC5XXX_REG_R63,
	MPC5XXX_REG_PC,
	MPC5XXX_REG_MSR,
	MPC5XXX_REG_CND,
	MPC5XXX_REG_LR,
	MPC5XXX_REG_CNT,
	MPC5XXX_REG_XER,
	MPC5XXX_REG_MQ,
	MPC5XXX_REG_R71,
};

struct mpc5xxx_comparator {
	int used;
	uint32_t bp_value;
	uint32_t reg_address;
};

struct mpc5xxx_common {
	int common_magic;
	struct mpc5xxx_jtag jtag;
	struct reg_cache *core_cache;
	uint32_t core_regs[MPC5XXX_NUMCOREREGS];

	int bp_scanned;
	int num_inst_bpoints;
	int num_data_bpoints;
	int num_inst_bpoints_avail;
	int num_data_bpoints_avail;
	struct mpc5xxx_comparator *inst_break_list;
	struct mpc5xxx_comparator *data_break_list;
	int jtag_state;
	uint32_t saved_ctl ;
	uint32_t saved_msr ;
	uint32_t ctl_on_entry ;
	uint32_t msr_on_entry ;
};

static inline struct mpc5xxx_common *
target_to_mpc5xxx(struct target *target)
{
	return (struct mpc5xxx_common *)target->arch_info;
}

struct mpc5xxx_core_reg {
	uint32_t num;
	struct target *target;
	struct mpc5xxx_common *mpc56xx_common;
};

extern const char * const mpc5xxx_core_reg_list[];
extern const struct mpc5xxx_core_reg
	mpc5xxx_core_reg_list_arch_info[];

extern const struct reg_arch_type mpc5xxx_reg_type;

int mpc5xxx_read_core_reg(struct target *target, int num);
int mpc5xxx_write_core_reg(struct target *target, int num);
int mpc5xxx_get_core_reg(struct reg *reg);
int mpc5xxx_set_core_reg(struct reg *reg, uint8_t *buf);
struct reg_cache *mpc5xxx_build_reg_cache(struct target *target);
int mpc5xxx_assert_reset(struct target *target);
int mpc5xxx_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer);
int mpc5xxx_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer);
int mpc5xxx_init_target(struct command_context *cmd_ctx,
	struct target *target);
int mpc5xxx_examine(struct target *target);
int mpc5xxx_arch_state(struct target *target);
int mpc5xxx_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
	int *reg_list_size, enum target_register_class reg_class);

int mpc5xxx_jtag_read_memory32(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, int count, uint32_t *buffer);
int mpc5xxx_jtag_read_memory16(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, int count, uint16_t *buffer);
int mpc5xxx_jtag_read_memory8(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, int count, uint8_t *buffer);

int mpc5xxx_jtag_write_memory32(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, int count, const uint32_t *buffer);
int mpc5xxx_jtag_write_memory16(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, int count, const uint16_t *buffer);
int mpc5xxx_jtag_write_memory8(struct mpc5xxx_jtag *jtag_info,
		uint32_t addr, int count, const uint8_t *buffer);

#endif
