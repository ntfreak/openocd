/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#ifdef _WIN32
#include <windows.h>
#else
#include <signal.h>
#endif

#include <helper/log.h>
#include <helper/time_support.h>
#include "aice_port.h"
#include "aice_pipe.h"

#define AICE_PIPE_MAXLINE 8192

#ifdef _WIN32
PROCESS_INFORMATION proc_info;

static HANDLE aice_pipe_output[2];
static HANDLE aice_pipe_input[2];

static int aice_pipe_write(const void *buffer, int count)
{
	BOOL success;
	DWORD written;

	success = WriteFile(aice_pipe_output[1], buffer, count, &written, NULL);
	if (!success) {
		LOG_ERROR("(WIN32) write to pipe failed, error code: 0x%08l" PRIx32, GetLastError());
		return -1;
	}

	return written;
}

static int aice_pipe_read(void *buffer, int count)
{
	BOOL success;
	DWORD has_read;

	success = ReadFile(aice_pipe_input[0], buffer, count, &has_read, NULL);
	if (!success || (has_read == 0)) {
		LOG_ERROR("(WIN32) read from pipe failed, error code: 0x%08l" PRIx32, GetLastError());
		return -1;
	}

	return has_read;
}

static int aice_pipe_child_init(struct aice_port_param_s *param)
{
	STARTUPINFO start_info;
	BOOL success;

	ZeroMemory(&proc_info, sizeof(PROCESS_INFORMATION));
	ZeroMemory(&start_info, sizeof(STARTUPINFO));
	start_info.cb = sizeof(STARTUPINFO);
	start_info.hStdError = aice_pipe_input[1];
	start_info.hStdOutput = aice_pipe_input[1];
	start_info.hStdInput = aice_pipe_output[0];
	start_info.dwFlags |= STARTF_USESTDHANDLES;

	success = CreateProcess(NULL,
			param->adapter_name,
			NULL,
			NULL,
			TRUE,
			0,
			NULL,
			NULL,
			&start_info,
			&proc_info);

	if (!success) {
		LOG_ERROR("Create new process failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_pipe_parent_init(struct aice_port_param_s *param)
{
	/* send open to adapter */
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_OPEN;
	set_u16(command + 1, param->vid);
	set_u16(command + 3, param->pid);

	if (aice_pipe_write(command, 5) != 5) {
		LOG_ERROR("write failed\n");
		return ERROR_FAIL;
	}

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0) {
		LOG_ERROR("read failed\n");
		return ERROR_FAIL;
	}

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_open(struct aice_port_param_s *param)
{
	SECURITY_ATTRIBUTES attribute;

	attribute.nLength = sizeof(SECURITY_ATTRIBUTES);
	attribute.bInheritHandle = TRUE;
	attribute.lpSecurityDescriptor = NULL;

	if (!CreatePipe(&aice_pipe_output[0], &aice_pipe_output[1],
				&attribute, AICE_PIPE_MAXLINE)) {
		LOG_ERROR("Create pipes failed");
		return ERROR_FAIL;
	}
	if (!CreatePipe(&aice_pipe_input[0], &aice_pipe_input[1],
				&attribute, AICE_PIPE_MAXLINE)) {
		LOG_ERROR("Create pipes failed");
		return ERROR_FAIL;
	}

	/* do not inherit aice_pipe_output[1] & aice_pipe_input[0] to child process */
	if (!SetHandleInformation(aice_pipe_output[1], HANDLE_FLAG_INHERIT, 0))
		return ERROR_FAIL;
	if (!SetHandleInformation(aice_pipe_input[0], HANDLE_FLAG_INHERIT, 0))
		return ERROR_FAIL;

	aice_pipe_child_init(param);

	aice_pipe_parent_init(param);

	return ERROR_OK;
}

#else

static int aice_pipe_output[2];
static int aice_pipe_input[2];

static int aice_pipe_write(const void *buffer, int count)
{
	if (write(aice_pipe_output[1], buffer, count) != count) {
		LOG_ERROR("write to pipe failed");
		return -1;
	}

	return count;
}

static int aice_pipe_read(void *buffer, int count)
{
	int n;
	int64_t then, cur;

	then = timeval_ms();

	while (1) {
		n = read(aice_pipe_input[0], buffer, count);

		if ((n == -1) && (errno == EAGAIN)) {
			cur = timeval_ms();
			if (cur - then > 500)
				keep_alive();
			continue;
		} else if (n > 0)
			break;
		else {
			LOG_ERROR("read from pipe failed");
			break;
		}
	}

	return n;
}

static int aice_pipe_child_init(struct aice_port_param_s *param)
{
	close(aice_pipe_output[1]);
	close(aice_pipe_input[0]);

	if (aice_pipe_output[0] != STDIN_FILENO) {
		if (dup2(aice_pipe_output[0], STDIN_FILENO) != STDIN_FILENO) {
			LOG_ERROR("Map aice_pipe to STDIN failed");
			return ERROR_FAIL;
		}
		close(aice_pipe_output[0]);
	}

	if (aice_pipe_input[1] != STDOUT_FILENO) {
		if (dup2(aice_pipe_input[1], STDOUT_FILENO) != STDOUT_FILENO) {
			LOG_ERROR("Map aice_pipe to STDOUT failed");
			return ERROR_FAIL;
		}
		close(aice_pipe_input[1]);
	}

	if (execl(param->adapter_name, param->adapter_name, (char *)0) < 0) {
		LOG_ERROR("Execute aice_pipe failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_pipe_parent_init(struct aice_port_param_s *param)
{
	close(aice_pipe_output[0]);
	close(aice_pipe_input[1]);

	/* set read end of pipe as non-blocking */
	if (fcntl(aice_pipe_input[0], F_SETFL, O_NONBLOCK))
		return ERROR_FAIL;

	/* send open to adapter */
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_OPEN;
	set_u16(command + 1, param->vid);
	set_u16(command + 3, param->pid);

	if (aice_pipe_write(command, 5) != 5) {
		LOG_ERROR("write failed\n");
		return ERROR_FAIL;
	}

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0) {
		LOG_ERROR("read failed\n");
		return ERROR_FAIL;
	}

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static void sig_pipe(int signo)
{
	exit(1);
}

static int aice_pipe_open(struct aice_port_param_s *param)
{
	pid_t pid;

	if (signal(SIGPIPE, sig_pipe) == SIG_ERR) {
		LOG_ERROR("Register SIGPIPE handler failed");
		return ERROR_FAIL;
	}

	if (pipe(aice_pipe_output) < 0 || pipe(aice_pipe_input) < 0) {
		LOG_ERROR("Create pipes failed");
		return ERROR_FAIL;
	}

	pid = fork();
	if (pid < 0) {
		LOG_ERROR("Fork new process failed");
		return ERROR_FAIL;
	} else if (pid == 0) {
		if (aice_pipe_child_init(param) != ERROR_OK) {
			LOG_ERROR("AICE_PIPE child process initial error");
			return ERROR_FAIL;
		} else {
			if (aice_pipe_parent_init(param) != ERROR_OK) {
				LOG_ERROR("AICE_PIPE parent process initial error");
				return ERROR_FAIL;
			}
		}
	}

	return ERROR_OK;
}
#endif

static int aice_pipe_close(void)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_CLOSE;

	if (aice_pipe_write(command, 1) != 1)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK) {
#ifdef _WIN32
		WaitForSingleObject(proc_info.hProcess, INFINITE);
		CloseHandle(proc_info.hProcess);
		CloseHandle(proc_info.hThread);
#endif
		return ERROR_OK;
	} else
		return ERROR_FAIL;
}

static int aice_pipe_idcode(uint32_t *idcode, uint8_t *num_of_idcode)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_IDCODE;

	if (aice_pipe_write(command, 1) != 1)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	*num_of_idcode = line[0];

	if ((*num_of_idcode == 0) || (*num_of_idcode >= 16))
		return ERROR_FAIL;

	for (int i = 0 ; i < *num_of_idcode ; i++)
		idcode[i] = get_u32(line + i * 4 + 1);

	return ERROR_OK;
}

static int aice_pipe_state(uint32_t coreid, enum aice_target_state_s *state)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_STATE;

	if (aice_pipe_write(command, 1) != 1)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	*state = (enum aice_target_state_s)line[0];

	return ERROR_OK;
}

static int aice_pipe_reset(void)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_RESET;

	if (aice_pipe_write(command, 1) != 1)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_assert_srst(uint32_t coreid, enum aice_srst_type_s srst)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_ASSERT_SRST;
	command[1] = srst;

	if (aice_pipe_write(command, 2) != 2)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_run(uint32_t coreid)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_RUN;

	if (aice_pipe_write(command, 1) != 1)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_halt(uint32_t coreid)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_HALT;

	if (aice_pipe_write(command, 1) != 1)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_read_reg(uint32_t coreid, uint32_t num, uint32_t *val)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_READ_REG;
	set_u32(command + 1, num);

	if (aice_pipe_write(command, 5) != 5)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	*val = get_u32(line);

	return ERROR_OK;
}

static int aice_pipe_write_reg(uint32_t coreid, uint32_t num, uint32_t val)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_WRITE_REG;
	set_u32(command + 1, num);
	set_u32(command + 5, val);

	if (aice_pipe_write(command, 9) != 9)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_read_reg_64(uint32_t coreid, uint32_t num, uint64_t *val)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_READ_REG_64;
	set_u32(command + 1, num);

	if (aice_pipe_write(command, 5) != 5)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	*val = (((uint64_t)get_u32(line + 4)) << 32) | get_u32(line);

	return ERROR_OK;
}

static int aice_pipe_write_reg_64(uint32_t coreid, uint32_t num, uint64_t val)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_WRITE_REG_64;
	set_u32(command + 1, num);
	set_u32(command + 5, val & 0xFFFFFFFF);
	set_u32(command + 9, (val >> 32) & 0xFFFFFFFF);

	if (aice_pipe_write(command, 13) != 9)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_step(uint32_t coreid)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_STEP;

	if (aice_pipe_write(command, 1) != 1)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_read_mem_unit(uint32_t coreid, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_READ_MEM_UNIT;
	set_u32(command + 1, addr);
	set_u32(command + 5, size);
	set_u32(command + 9, count);

	if (aice_pipe_write(command, 13) != 13)
		return ERROR_FAIL;

	if (aice_pipe_read(buffer, size * count) < 0)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_pipe_write_mem_unit(uint32_t coreid, uint32_t addr, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_WRITE_MEM_UNIT;
	set_u32(command + 1, addr);
	set_u32(command + 5, size);
	set_u32(command + 9, count);

	/* WRITE_MEM_UNIT|addr|size|count|data */
	memcpy(command + 13, buffer, size * count);

	if (aice_pipe_write(command, 13 + size * count) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_pipe_read_mem_bulk(uint32_t coreid, uint32_t addr,
		uint32_t length, uint8_t *buffer)
{
	char line[AICE_PIPE_MAXLINE + 1];
	char command[AICE_PIPE_MAXLINE];
	uint32_t remain_len = length;
	uint32_t prepare_len;
	char *received_line;
	uint32_t received_len;
	int read_len;

	command[0] = AICE_READ_MEM_BULK;
	set_u32(command + 1, addr);
	set_u32(command + 5, length);

	if (aice_pipe_write(command, 9) < 0)
		return ERROR_FAIL;

	while (remain_len > 0) {
		if (remain_len > AICE_PIPE_MAXLINE)
			prepare_len = AICE_PIPE_MAXLINE;
		else
			prepare_len = remain_len;

		prepare_len++;
		received_len = 0;
		received_line = line;
		do {
			read_len = aice_pipe_read(received_line, prepare_len - received_len);
			if (read_len < 0)
				return ERROR_FAIL;
			received_line += read_len;
			received_len += read_len;
		} while (received_len < prepare_len);

		if (line[0] != AICE_OK)
			return ERROR_FAIL;

		prepare_len--;
		memcpy(buffer, line + 1, prepare_len);
		remain_len -= prepare_len;
		buffer += prepare_len;
	}

	return ERROR_OK;
}

static int aice_pipe_write_mem_bulk(uint32_t coreid, uint32_t addr,
		uint32_t length, const uint8_t *buffer)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE + 4];
	uint32_t remain_len = length;
	uint32_t written_len = 0;
	uint32_t write_len;

	command[0] = AICE_WRITE_MEM_BULK;
	set_u32(command + 1, addr);
	set_u32(command + 5, length);

	/* Send command first */
	if (aice_pipe_write(command, 9) < 0)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_ERROR)
		return ERROR_FAIL;

	while (remain_len > 0) {
		if (remain_len > AICE_PIPE_MAXLINE)
			write_len = AICE_PIPE_MAXLINE;
		else
			write_len = remain_len;

		set_u32(command, write_len);
		memcpy(command + 4, buffer + written_len, write_len); /* data only */

		if (aice_pipe_write(command, write_len + 4) < 0)
			return ERROR_FAIL;

		if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
			return ERROR_FAIL;

		if (line[0] == AICE_ERROR)
			return ERROR_FAIL;

		remain_len -= write_len;
		written_len += write_len;
	}

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_read_debug_reg(uint32_t coreid, uint32_t addr, uint32_t *val)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_READ_DEBUG_REG;
	set_u32(command + 1, addr);

	if (aice_pipe_write(command, 5) != 5)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	*val = get_u32(line);

	return ERROR_OK;
}

static int aice_pipe_write_debug_reg(uint32_t coreid, uint32_t addr, const uint32_t val)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_WRITE_DEBUG_REG;
	set_u32(command + 1, addr);
	set_u32(command + 5, val);

	if (aice_pipe_write(command, 9) != 9)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_set_jtag_clock(uint32_t a_clock)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_SET_JTAG_CLOCK;
	set_u32(command + 1, a_clock);

	if (aice_pipe_write(command, 5) != 5)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_memory_access(uint32_t coreid, enum nds_memory_access access_channel)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_MEMORY_ACCESS;
	set_u32(command + 1, access_channel);

	if (aice_pipe_write(command, 5) != 5)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_memory_mode(uint32_t coreid, enum nds_memory_select mem_select)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_MEMORY_MODE;
	set_u32(command + 1, mem_select);

	if (aice_pipe_write(command, 5) != 5)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_read_tlb(uint32_t coreid, target_addr_t virtual_address,
		target_addr_t *physical_address)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_READ_TLB;
	set_u32(command + 1, virtual_address);

	if (aice_pipe_write(command, 5) != 5)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK) {
		*physical_address = get_u32(line + 1);
		return ERROR_OK;
	} else
		return ERROR_FAIL;
}

static int aice_pipe_cache_ctl(uint32_t coreid, uint32_t subtype, uint32_t address)
{
	char line[AICE_PIPE_MAXLINE];
	char command[AICE_PIPE_MAXLINE];

	command[0] = AICE_CACHE_CTL;
	set_u32(command + 1, subtype);
	set_u32(command + 5, address);

	if (aice_pipe_write(command, 9) != 9)
		return ERROR_FAIL;

	if (aice_pipe_read(line, AICE_PIPE_MAXLINE) < 0)
		return ERROR_FAIL;

	if (line[0] == AICE_OK)
		return ERROR_OK;
	else
		return ERROR_FAIL;
}

static int aice_pipe_set_retry_times(uint32_t a_retry_times)
{
	return ERROR_OK;
}

/** */
struct aice_port_api_s aice_pipe = {
	/** */
	.open = aice_pipe_open,
	/** */
	.close = aice_pipe_close,
	/** */
	.idcode = aice_pipe_idcode,
	/** */
	.set_jtag_clock = aice_pipe_set_jtag_clock,
	/** */
	.state = aice_pipe_state,
	/** */
	.reset = aice_pipe_reset,
	/** */
	.assert_srst = aice_pipe_assert_srst,
	/** */
	.run = aice_pipe_run,
	/** */
	.halt = aice_pipe_halt,
	/** */
	.step = aice_pipe_step,
	/** */
	.read_reg = aice_pipe_read_reg,
	/** */
	.write_reg = aice_pipe_write_reg,
	/** */
	.read_reg_64 = aice_pipe_read_reg_64,
	/** */
	.write_reg_64 = aice_pipe_write_reg_64,
	/** */
	.read_mem_unit = aice_pipe_read_mem_unit,
	/** */
	.write_mem_unit = aice_pipe_write_mem_unit,
	/** */
	.read_mem_bulk = aice_pipe_read_mem_bulk,
	/** */
	.write_mem_bulk = aice_pipe_write_mem_bulk,
	/** */
	.read_debug_reg = aice_pipe_read_debug_reg,
	/** */
	.write_debug_reg = aice_pipe_write_debug_reg,

	/** */
	.memory_access = aice_pipe_memory_access,
	/** */
	.memory_mode = aice_pipe_memory_mode,

	/** */
	.read_tlb = aice_pipe_read_tlb,

	/** */
	.cache_ctl = aice_pipe_cache_ctl,

	/** */
	.set_retry_times = aice_pipe_set_retry_times,
};
