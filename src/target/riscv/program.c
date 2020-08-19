/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/register.h"
#include "riscv.h"
#include "program.h"
#include "helper/log.h"

#include "asm.h"
#include "encoding.h"

/* Program interface. */
int riscv_program_init(struct riscv_program *p, struct target *target)
{
	memset(p, 0, sizeof(*p));
	p->target = target;
	p->instruction_count = 0;
	p->target_xlen = riscv_xlen(target);
	for (size_t i = 0; i < RISCV_REGISTER_COUNT; ++i)
		p->writes_xreg[i] = 0;

	for (size_t i = 0; i < RISCV_MAX_DEBUG_BUFFER_SIZE; ++i)
		p->debug_buffer[i] = -1;

	return ERROR_OK;
}

int riscv_program_write(struct riscv_program *program)
{
	for (unsigned i = 0; i < program->instruction_count; ++i) {
		LOG_DEBUG("debug_buffer[%02x] = DASM(0x%08x)", i, program->debug_buffer[i]);
		if (riscv_write_debug_buffer(program->target, i,
					program->debug_buffer[i]) != ERROR_OK)
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

/** Add ebreak and execute the program. */
int riscv_program_exec(struct riscv_program *p, struct target *t)
{
	keep_alive();

	riscv_reg_t saved_registers[GDB_REGNO_XPR31 + 1];
	for (size_t i = GDB_REGNO_ZERO + 1; i <= GDB_REGNO_XPR31; ++i) {
		if (p->writes_xreg[i]) {
			LOG_DEBUG("Saving register %d as used by program", (int)i);
			int result = riscv_get_register(t, &saved_registers[i], i);
			if (result != ERROR_OK)
				return result;
		}
	}

	if (riscv_program_ebreak(p) != ERROR_OK) {
		LOG_ERROR("Unable to write ebreak");
		for (size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
			LOG_ERROR("ram[%02x]: DASM(0x%08" PRIx32 ") [0x%08" PRIx32 "]",
					(int)i, p->debug_buffer[i], p->debug_buffer[i]);
		return ERROR_FAIL;
	}

	if (riscv_program_write(p) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_execute_debug_buffer(t) != ERROR_OK) {
		LOG_DEBUG("Unable to execute program %p", p);
		return ERROR_FAIL;
	}

	for (size_t i = 0; i < riscv_debug_buffer_size(p->target); ++i)
		if (i >= riscv_debug_buffer_size(p->target))
			p->debug_buffer[i] = riscv_read_debug_buffer(t, i);

	for (size_t i = GDB_REGNO_ZERO; i <= GDB_REGNO_XPR31; ++i)
		if (p->writes_xreg[i])
			riscv_set_register(t, i, saved_registers[i]);

	return ERROR_OK;
}

int riscv_program_sdr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sd(d, b, offset));
}

int riscv_program_swr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sw(d, b, offset));
}

int riscv_program_shr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sh(d, b, offset));
}

int riscv_program_sbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, sb(d, b, offset));
}

int riscv_program_ldr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, ld(d, b, offset));
}

int riscv_program_lwr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lw(d, b, offset));
}

int riscv_program_lhr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lh(d, b, offset));
}

int riscv_program_lbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int offset)
{
	return riscv_program_insert(p, lb(d, b, offset));
}

int riscv_program_csrrsi(struct riscv_program *p, enum gdb_regno d, unsigned int z, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0 && csr <= GDB_REGNO_CSR4095);
	return riscv_program_insert(p, csrrsi(d, z, csr - GDB_REGNO_CSR0));
}

int riscv_program_csrrci(struct riscv_program *p, enum gdb_regno d, unsigned int z, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0 && csr <= GDB_REGNO_CSR4095);
	return riscv_program_insert(p, csrrci(d, z, csr - GDB_REGNO_CSR0));
}

int riscv_program_csrr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0 && csr <= GDB_REGNO_CSR4095);
	return riscv_program_insert(p, csrrs(d, GDB_REGNO_ZERO, csr - GDB_REGNO_CSR0));
}

int riscv_program_csrw(struct riscv_program *p, enum gdb_regno s, enum gdb_regno csr)
{
	assert(csr >= GDB_REGNO_CSR0);
	return riscv_program_insert(p, csrrw(GDB_REGNO_ZERO, s, csr - GDB_REGNO_CSR0));
}

int riscv_program_fence_i(struct riscv_program *p)
{
	return riscv_program_insert(p, fence_i());
}

int riscv_program_fence(struct riscv_program *p)
{
	return riscv_program_insert(p, fence());
}

int riscv_program_ebreak(struct riscv_program *p)
{
	struct target *target = p->target;
	RISCV_INFO(r);
	if (p->instruction_count == riscv_debug_buffer_size(p->target) &&
			r->impebreak) {
		return ERROR_OK;
	}
	return riscv_program_insert(p, ebreak());
}

int riscv_program_addi(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int16_t u)
{
	return riscv_program_insert(p, addi(d, s, u));
}

int riscv_program_insert(struct riscv_program *p, riscv_insn_t i)
{
	if (p->instruction_count >= riscv_debug_buffer_size(p->target)) {
		LOG_ERROR("Unable to insert instruction:");
		LOG_ERROR("  instruction_count=%d", (int)p->instruction_count);
		LOG_ERROR("  buffer size      =%d", (int)riscv_debug_buffer_size(p->target));
		return ERROR_FAIL;
	}

	p->debug_buffer[p->instruction_count] = i;
	p->instruction_count++;
	return ERROR_OK;
}
