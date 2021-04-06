/* SPDX-License-Identifier: (GPL-2.0-or-later OR BSD-2-Clause) */
/*----------------------------------------------------------------------------
 * Copyright 2020-2021 Cadence Design Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *----------------------------------------------------------------------------
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *----------------------------------------------------------------------------
*/

/*!
 * @file
 *
 * @brief the virtual debug interface provides a connection between a sw debugger
 * and the simulated, emulated core over a soft connection, implemented by DPI
 * The vdebug debug driver currently supports  the JTAG transport
 *
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#ifdef HAVE_UNISTD_H
#include <unistd.h>          /* close */
#endif
#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif
#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif
#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif
#endif
#include <stdio.h>
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "jtag/interface.h"
#include "jtag/commands.h"
#include "transport/transport.h"
#include "target/target.h"
#include "target/target_type.h"
#include "helper/time_support.h"
#include "helper/replacements.h"
#include "helper/log.h"

#define VD_VERSION 42
#define VD_BUFFER_LEN 4024
#define VD_CHEADER_LEN 24
#define VD_SHEADER_LEN 16

#define VD_MAX_MEMORIES 4
#define VD_POLL_INTERVAL 500
#define VD_SCALE_PSTOMS 1000000000

/**
 * @brief List of transactor types
 */
enum {
	VD_BFM_JTDP   = 0x0001,  /* transactor DAP JTAG DP */
	VD_BFM_SWDP   = 0x0002,  /* transactor DAP SWD DP */
	VD_BFM_AHB    = 0x0003,  /* transactor AMBA AHB */
	VD_BFM_APB    = 0x0004,  /* transactor AMBA APB */
	VD_BFM_AXI    = 0x0005,  /* transactor AMBA AXI */
	VD_BFM_JTAG   = 0x0006,  /* transactor serial JTAG */
	VD_BFM_SWD    = 0x0007,  /* transactor serial SWD */
};

/**
 * @brief List of signals that can be read or written by the debugger
 */
enum {
	VD_SIG_TCK    = 0x0001,  /* JTAG clock; tclk */
	VD_SIG_TDI    = 0x0002,  /* JTAG TDI;   tdi */
	VD_SIG_TMS    = 0x0004,  /* JTAG TMS;   tms */
	VD_SIG_RESET  = 0x0008,  /* DUT reset;  rst */
	VD_SIG_TRST   = 0x0010,  /* JTAG Reset; trstn */
	VD_SIG_TDO    = 0x0020,  /* JTAG TDO;   tdo */
	VD_SIG_POWER  = 0x0100,  /* BFM power;  bfm_up */
	VD_SIG_TCKDIV = 0x0200,  /* JTAG clock divider; tclkdiv */
	VD_SIG_BUF    = 0x1000,  /* memory buffer; mem */
};

/**
 * @brief List of errors
 */
enum {
	VD_ERR_NONE       = 0x0000,  /* no error */
	VD_ERR_NOT_IMPL   = 0x0100,  /* feature not implemented */
	VD_ERR_USAGE      = 0x0101,  /* incorrect usage */
	VD_ERR_PARAM      = 0x0102,  /* incorrect parameter */
	VD_ERR_CONFIG     = 0x0107,  /* incorrect configuration */
	VD_ERR_NO_MEMORY  = 0x0104,  /* out of memory */
	VD_ERR_SHM_OPEN   = 0x010a,  /* cannot open shared memory */
	VD_ERR_SHM_MAP    = 0x010b,  /* cannot map shared memory */
	VD_ERR_SOC_OPEN   = 0x011a,  /* cannot open socket */
	VD_ERR_SOC_OPT    = 0x011b,  /* cannot set socket option */
	VD_ERR_SOC_ADDR   = 0x011c,  /* cannot resolve host address */
	VD_ERR_SOC_CONN   = 0x011d,  /* cannot connect to host */
	VD_ERR_SOC_SEND   = 0x011e,  /* error sending data on socket */
	VD_ERR_SOC_RECV   = 0x011f,  /* error receiving data from socket */
	VD_ERR_LOCKED     = 0x0202,  /* device locked */
	VD_ERR_NOT_RUN    = 0x0204,  /* transactor not running */
	VD_ERR_NOT_OPEN   = 0x0205,  /* transactor not open/connected */
	VD_ERR_LICENSE    = 0x0206,  /* cannot check out the license */
	VD_ERR_VERSION    = 0x0207,  /* transactor version mismatch */
	VD_ERR_TIME_OUT   = 0x0301,  /* time out, waiting */
	VD_ERR_NO_POWER   = 0x0302,  /* power out error */
	VD_ERR_BUS_ERROR  = 0x0304,  /* bus protocol error, like pslverr */
	VD_ERR_NO_ACCESS  = 0x0306,  /* no access to an object */
	VD_ERR_INV_HANDLE = 0x0307,  /* invalid object handle */
	VD_ERR_INV_SCOPE  = 0x0308,  /* invalid scope */
};

enum {
	VD_CMD_OPEN       = 0x01,
	VD_CMD_CLOSE      = 0x02,
	VD_CMD_CONNECT    = 0x04,
	VD_CMD_DISCONNECT = 0x05,
	VD_CMD_WAIT       = 0x09,
	VD_CMD_SIGSET     = 0x0a,
	VD_CMD_SIGGET     = 0x0b,
	VD_CMD_JTAGCLOCK  = 0x0f,
	VD_CMD_JTAGSHTAP  = 0x1a,
	VD_CMD_MEMOPEN    = 0x21,
	VD_CMD_MEMCLOSE   = 0x22,
	VD_CMD_MEMWRITE   = 0x23,
};

enum {
	VD_BATCH_NO       = 0,
	VD_BATCH_WO       = 1,
	VD_BATCH_WR       = 2,
};

struct vd_shm {
	struct {                     /* VD_CHEADER_LEN written by client */
		uint8_t cmd;             /* 000; command */
		uint8_t type;            /* 001; interface type */
		uint16_t waddr;          /* 002; write pointer */
		uint16_t wbytes;         /* 004; data bytes */
		uint16_t rbytes;         /* 006; data bytes to read */
		uint16_t wwords;         /* 008; data words */
		uint16_t rwords;         /* 00a; data words to read */
		uint32_t rwdata;         /* 00c; read/write data */
		uint32_t offset;         /* 010; address offset */
		uint16_t offseth;        /* 014; address offset 47:32 */
		uint16_t wid;            /* 016; request id*/
	};
	union {                      /* 018; */
		uint8_t wd8[VD_BUFFER_LEN];
		uint32_t wd32[VD_BUFFER_LEN/4];
		uint64_t wd64[VD_BUFFER_LEN/8];
	};
	struct {                     /* VD_SHEADER_LEN written by server */
		uint16_t rid;            /* fd0: request id read */
		uint16_t awords;         /* fd2: actual data words read back */
		int32_t  status;         /* fd4; */
		uint64_t duttime;        /* fd8; */
	};
	union {                      /* fe0: */
		uint8_t rd8[VD_BUFFER_LEN];
		uint32_t rd32[VD_BUFFER_LEN/4];
		uint64_t rd64[VD_BUFFER_LEN/8];
	};
	uint32_t state;              /* 1f98; connection state */
	uint32_t count;              /* 1f9c; */
	uint8_t dummy[96];           /* 1fa0; 48+40B+8B; */
};

struct vd_client {
	uint8_t trans_batch;
	bool trans_first;
	bool trans_last;
	uint8_t mem_ndx;
	uint8_t buf_width;
	uint8_t addr_bits;
	uint8_t bfm_type;
	uint16_t sig_read;
	uint16_t sig_write;
	uint32_t bfm_period;
	uint32_t mem_base[VD_MAX_MEMORIES];
	uint32_t mem_size[VD_MAX_MEMORIES];
	uint32_t mem_width[VD_MAX_MEMORIES];
	uint32_t mem_depth[VD_MAX_MEMORIES];
	uint32_t server_port;
	uint32_t poll_cycles;
	uint32_t poll_min;
	uint32_t poll_max;
	uint32_t targ_time;
	int hsocket;
	char server_name[32];
	char bfm_path[128];
	char mem_path[VD_MAX_MEMORIES][128];
	enum target_state targ_state;
	struct target *targ;
	uint8_t *tdo;
};

struct vd_jtag_hdr {
	uint64_t dbits:24;
	uint64_t post:3;
	uint64_t pre:3;
	uint64_t cmd:2;
	uint64_t wlen:16;
	uint64_t rlen:16;
};

static struct vd_shm *pbuf;
static struct vd_client vdc;

static int socket_error(void)
{
#ifdef _WIN32
	return WSAGetLastError();
#else
	return errno;
#endif
}

static int socket_open(char *server_addr, uint32_t port)
{
	int hsock;
	int rc = 0;
	uint32_t buflen = sizeof(struct vd_shm); /* size of the send and rcv buffer */
	struct addrinfo *ainfo = NULL;
	struct addrinfo ahint = { 0, AF_INET, SOCK_STREAM, 0, 0, NULL, NULL, NULL };

#ifdef _WIN32
	hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (hsock == INVALID_SOCKET)
		rc = socket_error();
#else
	uint32_t rcvwat = VD_SHEADER_LEN;    /* size of the rcv header, as rcv min watermark */
	hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (hsock < 0)
		rc = errno;
	else if (setsockopt(hsock, SOL_SOCKET, SO_RCVLOWAT, &rcvwat, sizeof(rcvwat)) < 0)
		rc = errno;
#endif
	else if (setsockopt(hsock, SOL_SOCKET, SO_SNDBUF, (const char *)&buflen, sizeof(buflen)) < 0)
		rc = socket_error();
	else if (setsockopt(hsock, SOL_SOCKET, SO_RCVBUF, (const char *)&buflen, sizeof(buflen)) < 0)
		rc = socket_error();
	if (rc)
		LOG_ERROR("socket_open: cannot set socket option, error %d", rc);
	else if (getaddrinfo(server_addr, NULL, &ahint, &ainfo) != 0) {
		LOG_ERROR("socket_open: cannot resolve address %s, error %d", server_addr, socket_error());
		rc = VD_ERR_SOC_ADDR;
	} else {
		((struct sockaddr_in *)(ainfo->ai_addr))->sin_port = htons(port);
		if (connect(hsock, ainfo->ai_addr, sizeof(struct sockaddr)) < 0) {
			LOG_ERROR("socket_open: cannot connect to %s:%d, error %d", server_addr, port, socket_error());
			rc = VD_ERR_SOC_CONN;
		}
	}

	if (rc) {
		close_socket(hsock);
		hsock = 0;
	}

	if (ainfo)
		freeaddrinfo(ainfo);

	return hsock;
}

static int socket_receive(int hsock, struct vd_shm *pmem)
{
	int rc;
	uint16_t dreceived = 0;
	uint16_t offset = (uint8_t *)&pmem->rid - &pmem->cmd;
	uint16_t to_receive = VD_SHEADER_LEN + pmem->rbytes;
	char *pb = (char *)pmem;

	do {
		rc = recv(hsock, pb + offset, to_receive, 0);
		if (rc <= 0)
			break;
		else {              /* the data can come in pieces */
			to_receive -= rc;
			offset += rc;
		}
		LOG_DEBUG_IO("socket_receive: received %u, to receive %u", rc, to_receive);
		dreceived += rc;
	} while (rc > 0 && to_receive);

	if (rc <= 0)
		LOG_WARNING("socket_receive: recv failed, error %d", socket_error());
	else
		rc = dreceived;

	return rc;
}

static int socket_send(int hsock, struct vd_shm *pmem)
{
	int rc = send(hsock, (const char *)&pmem->cmd, VD_CHEADER_LEN + pmem->wbytes, 0);
	if (rc <= 0)
		LOG_WARNING("socket_send: send failed, error %d", socket_error());
	else
		LOG_DEBUG_IO("socket_send: sent %u, to send %u", rc, 0);

	return rc;
}

static uint32_t wait_server(int hsock, struct vd_shm *pmem)
{
	int rc;
	int st = socket_send(hsock, pmem);
	int rd = socket_receive(hsock, pmem);
	if (!hsock)
		rc = VD_ERR_SOC_OPEN;
	else if (st <= 0)
		rc = VD_ERR_SOC_SEND;
	else if (rd  <= 0)
		rc = VD_ERR_SOC_RECV;
	else {
		rc = pmem->status;
		LOG_DEBUG_IO("wait_server: cmd %02hx done, sent %d, rcvd %d, status %d",
					pmem->cmd, st, rd, rc);
	}

	return rc;
}

int exec_jtag_queue(int hsock, struct vd_shm *pm, uint32_t count)
{
	uint8_t  num_pre, num_post, tdi, tms;
	uint16_t num, anum, bytes, hwords, words, j;
	uint16_t req, rreq, waddr, rwords;
	int64_t ts, te;
	uint8_t *tdo;
	int rc;
	struct vd_jtag_hdr *hdr;

	req = rreq = waddr = rwords = 0;/* beginning of request */
	pm->wbytes = pm->wwords * 8;
	pm->rbytes = pm->rwords * 8;
	ts = timeval_ms();
	rc = wait_server(hsock, pm);
	while (!rc && (req < count)) {      /* loop over requests to read data and print out */
		hdr = (struct vd_jtag_hdr *)&pm->wd8[waddr * 4];
		hwords = hdr->wlen;
		words = hdr->rlen;
		anum = hdr->dbits;
		num_pre = hdr->pre;
		num_post = hdr->post;
		if (num_post)
			num = anum - num_pre - num_post + 1;
		else
			num = anum - num_pre;
		bytes = (num + 7) / 8;
		vdc.trans_last = ((uint32_t)(req + 1) < count ? 0 : 1);
		vdc.trans_first = (waddr ? 0 : 1);
		if (hdr->cmd == 3) { /* read */
			tdo = vdc.tdo;
			for (j = 0; j < bytes; j++) {
				tdo[j] = (pm->rd8[rwords * 8 + j] >> num_pre) | (pm->rd8[rwords * 8 + j + 1] << (8 - num_pre));
				LOG_DEBUG_IO("vd_jtag_shift_tap: %04x D0[%02x]:%02x", pm->wid - count + req, j, tdo[j]);
			}
			rwords += words;           /* read data offset */
		} else
			tdo = NULL;
		waddr += 2;                    /* waddr past header */
		tdi = (pm->wd8[waddr * 4] >> num_pre) | (pm->wd8[waddr * 4 + 1] << (8 - num_pre));
		tms = (pm->wd8[waddr * 4 + 4] >> num_pre) | (pm->wd8[waddr * 4 + 4 + 1] << (8 - num_pre));
		LOG_DEBUG("vd_jtag_shift_tap: %04x L:%02d O:%05x @%03x DI:%02x MS:%02x DO:%02x",
			pm->wid-count+req, num, ((vdc.trans_first << 14)|(vdc.trans_last << 15)),
			waddr - 2, tdi, tms, (tdo ? tdo[0] : 0xdd));
		waddr += hwords * 2;           /* start of next request */
		req += 1;
	}

	if (rc) {
		LOG_ERROR("exec_jtag_queue: Error 0x%x executing transaction", rc);
		rc = ERROR_FAIL;
	}

	te = timeval_ms();
	vdc.targ_time += (uint32_t)(te - ts);
	pm->offseth = 0;     /* reset buffer write address */
	pm->offset = 0;
	pm->rwords = 0;
	pm->waddr = 0;

	return rc;
}

static int vdebug_open(int hsock, struct vd_shm *pm, const char *path,
						uint32_t type, uint32_t period_ps, uint32_t sig_mask)
{
	int rc = VD_ERR_NOT_OPEN;

	pm->cmd = VD_CMD_OPEN;
	pm->wid = (uint16_t)VD_VERSION;    /* client version */
	pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
	rc = wait_server(hsock, pm);
	if (rc != 0)                   /* communication problem */
		LOG_ERROR("vdebug_open: Error 0x%x connecting to server", rc);
	else if (pm->rid < pm->wid) {
		LOG_ERROR("vdebug_open: server version %d too old for the client %d", pm->rid, pm->wid);
		pm->cmd = VD_CMD_CLOSE;    /* let server close the connection */
		wait_server(hsock, pm);
		rc = VD_ERR_VERSION;
	} else {
		pm->cmd = VD_CMD_CONNECT;
		pm->type = (uint8_t)type;      /* BFM type to connect to, here JTAG */
		pm->rwdata = sig_mask | VD_SIG_BUF | (VD_SIG_BUF << 16);
		pm->wbytes = (uint16_t)strlen(path) + 1;
		pm->rbytes = 12;
		pm->wid = 0;               /* reset wid for transaction ID */
		pm->wwords = pm->rwords = 0;
		memcpy(pm->wd8, path, pm->wbytes + 1);
		rc = wait_server(hsock, pm);
		vdc.sig_read = (uint16_t)(pm->rwdata >> 16);    /* signal read mask */
		vdc.sig_write = (uint16_t)pm->rwdata;   /* signal write mask */
		vdc.bfm_period = period_ps;
		vdc.buf_width = pm->rd32[0] / 8;/* access width in bytes */
		vdc.addr_bits = pm->rd32[2];    /* supported address bits */
	}

	if (rc) {
		LOG_ERROR("vdebug_open: Error 0x%x connecting to BFM %s", rc, path);
		rc = ERROR_FAIL;
	} else
		LOG_DEBUG("vdebug_open: %s type %0x, period %dps, buffer %dx%dB signals r%04xw%04x",
			path, type, vdc.bfm_period, VD_BUFFER_LEN / vdc.buf_width,
			vdc.buf_width, vdc.sig_read, vdc.sig_write);

	return rc;
}

static int vdebug_close(int hsock, struct vd_shm *pm, uint32_t type)
{
	pm->cmd = VD_CMD_DISCONNECT;
	pm->type = (uint8_t)type;          /* BFM type, here JTAG */
	pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
	wait_server(hsock, pm);
	pm->cmd = VD_CMD_CLOSE;
	pm->wid = (uint16_t)VD_VERSION;    /* client version */
	pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
	wait_server(hsock, pm);
	LOG_DEBUG("vdebug_close: type %0x", type);

	return ERROR_OK;
}

static int vdebug_wait(int hsock, struct vd_shm *pm, uint32_t cycles)
{
	int rc = ERROR_OK;

	if (cycles) {
		pm->cmd = VD_CMD_WAIT;
		pm->wbytes = 0;
		pm->rbytes = 0;
		pm->rwdata = cycles;  /* clock sycles to wait */
		rc = wait_server(hsock, pm);
	}

	if (rc) {
		LOG_ERROR("vdebug_wait: Error 0x%x waiting %d cycles", rc, cycles);
		rc = ERROR_FAIL;
	} else
		LOG_DEBUG("vdebug_wait: %d cycles", cycles);

	return rc;
}

static int vdebug_sig_set(int hsock, struct vd_shm *pm, uint32_t write_mask, uint32_t value)
{
	int rc;

	pm->cmd = VD_CMD_SIGSET;
	pm->wbytes = 0;
	pm->rbytes = 0;
	pm->rwdata = (write_mask << 16) | (value & 0xffff); /* mask and value of signals to set */
	rc = wait_server(hsock, pm);
	if (rc) {
		LOG_WARNING("vdebug_sig_set: Error 0x%x setting signals %04x", rc, write_mask);
		rc = ERROR_FAIL;
	} else
		LOG_DEBUG("vdebug_sig_set: setting signals %04x to %04x", write_mask, value);

	return rc;
}

static int vdebug_jtag_clock(int hsock, struct vd_shm *pm, uint32_t value)
{
	int rc;

	pm->cmd = VD_CMD_JTAGCLOCK;
	pm->wbytes = 0;
	pm->rbytes = 0;
	pm->rwdata = value;  /* divider value */
	rc = wait_server(hsock, pm);
	if (rc) {
		LOG_WARNING("vdebug_jtag_clock: Error 0x%x setting jtag_clock", rc);
		rc = ERROR_FAIL;
	} else
		LOG_DEBUG("vdebug_jtag_clock: setting jtag clock divider to %d", value);

	return rc;
}

static int vdebug_jtag_shift_tap(int hsock, struct vd_shm *pm, uint8_t num_pre,
								 const uint8_t tms_pre, uint32_t num, const uint8_t *tdi,
								 uint8_t num_post, const uint8_t tms_post, uint8_t *tdo,
								 uint8_t f_last)
{
	const uint32_t tobits = 8;
	uint16_t i, j;
	uint16_t bytes, hwords, anum, words, waddr;
	int rc = 0;
	struct vd_jtag_hdr *hdr;

	pm->cmd = VD_CMD_JTAGSHTAP;
	vdc.trans_last = f_last || (vdc.trans_batch == VD_BATCH_NO) || (tdo != NULL);
	if (vdc.trans_first)
		waddr = 0;             /* reset buffer offset */
	else
		waddr = pm->offseth;   /* continue from the previous transaction */
	if (num_post)          /* actual number of bits to shift */
		anum = num + num_pre + num_post - 1;
	else
		anum = num + num_pre;
	hwords = (anum + 4 * vdc.buf_width - 1)/(4 * vdc.buf_width); /* in 4B TDI/TMS words */
	words = (hwords + 1) / 2;    /* in 8B TDO words to read */
	bytes = (num + 7) / 8;       /* data only portion in bytes */
	/* buffer overflow check and flush */
	if (waddr + 2 + 2 * hwords + 16 > VD_BUFFER_LEN/4)
		vdc.trans_last = 1;        /* force flush within 64B of buffer end */
	else if (waddr + 2 + 2 * hwords > VD_BUFFER_LEN/4) {
		/* this req does not fit, discard it */
		LOG_WARNING("vdebug_jtag_shift_tap: %04x L:%02d O:%05x @%04x too many bits, Error",
			pm->wid, anum, ((vdc.trans_first << 14)|(vdc.trans_last << 15)), waddr);
		rc = ERROR_FAIL;
	}

	if (!rc && anum) {
		hdr = (struct vd_jtag_hdr *)&pm->wd8[4 * waddr]; /* 8 bytes header */
		hdr->cmd = (tdo ? 3 : 1); /* R and W bits */
		hdr->pre = num_pre;
		hdr->post = num_post;
		hdr->dbits = anum;
		hdr->wlen = hwords;
		hdr->rlen = words;
		pm->wid++;               /* transaction ID */
		waddr += 2;              /* waddr past header */
		/* TDI/TMS data follows as 32 bit word pairs {TMS,TDI} */
		pm->wd8[4 * waddr] = (tdi ? (tdi[0] << num_pre) : 0);
		pm->wd8[4 * waddr + 4] = tms_pre;    /* init with tms_pre */
		if (num + num_pre <= 8)            /* and tms_post for num <=4 */
			pm->wd8[4 * waddr + 4] |= (tms_post << (num+num_pre - 1));
		for (i = 1, j = 4 * waddr; i < bytes; i++) {
			if (i == bytes - 1 && num + num_pre <= bytes*tobits)
				pm->wd8[j + i + 4] = tms_post << ((num + num_pre - 1) % 8);
			else
				pm->wd8[j + i + 4] = 0x0;/* placing 4 bytes of TMS bits into high word */
			if (!tdi)             /* placing 4 bytes of TDI bits into low word */
				pm->wd8[j + i] = 0x0;
			else
				pm->wd8[j + i] = (tdi[i] << num_pre) | (tdi[i - 1] >> (8 - num_pre));
			if (i % 4 == 3)
				j += 4;
		}

		if (tdi) {
			if (num + num_pre > bytes*tobits) /* in case 1 additional byte needed for TDI */
				pm->wd8[j + i] = (tdi[i - 1] >> (8 - num_pre)); /* put last TDI bits there */
		}

		if (num + num_pre <= bytes * tobits) /* in case no or 1 additional byte needed */
			pm->wd8[j + i + 4] = tms_post >> (8 - (num + num_pre - 1) % 8); /* may need to add higher part */
		/* in case exactly 1 additional byte needed */
		else if (num + num_pre > bytes * tobits && anum <= (bytes + 1) * tobits)
			pm->wd8[j + i + 4] = tms_post << ((num + num_pre - 1) % 8); /* add whole tms_post */
		else {                           /* in case 2 additional bytes, tms_post split */
			pm->wd8[j + i + 4] = tms_post << ((num + num_pre - 1) % 8);/* add lower part of tms_post */
			if (i % 4 == 3)              /* next byte is in the next 32b word */
				pm->wd8[j + i + 4 + 5] = tms_post >> (8-(num + num_pre - 1) % 8); /* and higher part */
			else                         /* next byte is in the same 32b word */
				pm->wd8[j + i + 4 + 1] = tms_post >> (8-(num + num_pre - 1) % 8); /* and higher part */
		}

		if (tdo) {
			pm->rwords += words;       /* keep track of the words to read */
			vdc.tdo = tdo;
		}
		pm->wwords = waddr / 2 + hwords;   /* payload size *2 to include both TDI and TMS data */
		pm->waddr++;
	}

	if (!waddr)                        /* flush issued, but buffer empty */
		;
	else if (!vdc.trans_last)          /* buffered request */
		pm->offseth = waddr + hwords * 2;  /* offset for next transaction, must be even */
	else                               /* execute batch of requests */
		rc = exec_jtag_queue(hsock, pm, pm->waddr);
	vdc.trans_first = vdc.trans_last; /* flush forces trans_first flag */

	return rc;
}

static int vdebug_mem_open(int hsock, struct vd_shm *pm, const char* path, uint8_t ndx)
{
	int rc = ERROR_OK;

	if (!path)
		return rc;

	pm->cmd = VD_CMD_MEMOPEN;
	pm->wbytes = (uint16_t)strlen(path) + 1;   /* includes terminating 0 */
	pm->rbytes = 8;
	pm->wwords = pm->rwords = 0;
	memcpy(pm->wd8, path, pm->wbytes);
	rc = wait_server(hsock, pm);
	if (rc) {
		LOG_WARNING("vdebug_mem_open: Error 0x%x opening memory %s", rc, path);
		rc = ERROR_FAIL;
	} else {
		vdc.mem_width[ndx] = pm->rd32[0] / 8;   /* memory width in bytes */
		vdc.mem_depth[ndx] = pm->rd32[1];       /* memory depth in words */
		LOG_DEBUG("vdebug_mem_open: %s memory %xx%dB, buffer %dx%dB",
			path, vdc.mem_depth[ndx], vdc.mem_width[ndx], VD_BUFFER_LEN / vdc.mem_width[ndx], vdc.mem_width[ndx]);
	}

	return rc;
}

static void vdebug_mem_close(int hsock, struct vd_shm *pm, uint8_t ndx)
{
	pm->cmd = VD_CMD_MEMCLOSE;
	pm->rwdata = ndx;        /* which memory */
	pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
	wait_server(hsock, pm);
	LOG_DEBUG("vdebug_mem_close: %s", vdc.mem_path[ndx]);
}

static int vdebug_poll(void *priv)
{
	int rc = ERROR_OK;
	int64_t ts, te;
	uint32_t cmdtime;
	struct target **t = (struct target **)priv;

	if (*t && vdc.targ_state != (*t)->state) {
		vdc.targ_state = (*t)->state;
		vdc.poll_cycles = vdc.poll_max; /* reset polling when target state changes */
	}

	if (vdc.targ_time < vdc.poll_min) {
		ts = timeval_ms();
		vdebug_wait(vdc.hsocket, pbuf, vdc.poll_cycles);
		te = timeval_ms();
		cmdtime = (uint32_t)(te - ts);
		if (cmdtime < ((vdc.poll_min + vdc.poll_max) / 3))
			vdc.poll_cycles *= 2;
		else if (cmdtime > ((vdc.poll_min + vdc.poll_max) * 2 / 3))
			vdc.poll_cycles /= 2;
	}

	LOG_DEBUG("poll after %ums in state %u; wait %u cycles in %ums",
		vdc.targ_time, vdc.targ_state, vdc.poll_cycles, cmdtime);
	vdc.targ_time = 0;                 /* reset targte time counter */

	return rc;
}

static int vdebug_init(void)
{
	uint32_t sig_mask;
	int rc;

	vdc.hsocket = socket_open(vdc.server_name, vdc.server_port);
	pbuf = (struct vd_shm *)calloc(1, sizeof(struct vd_shm));
	if (!pbuf) {
		close_socket(vdc.hsocket);
		vdc.hsocket = 0;
		LOG_ERROR("cannot allocate %lu bytes", sizeof(struct vd_shm));
		rc = ERROR_FAIL;
	} else if (vdc.hsocket <= 0) {
		free(pbuf);
		pbuf = NULL;
		LOG_ERROR("cannot connect to vdebug server %s:%d",
			vdc.server_name, vdc.server_port);
		rc = ERROR_FAIL;
	} else {
		vdc.trans_first = 1;
		vdc.poll_cycles = vdc.poll_max;
		sig_mask = VD_SIG_RESET | VD_SIG_TRST | VD_SIG_TCKDIV;
		rc = vdebug_open(vdc.hsocket, pbuf, vdc.bfm_path, vdc.bfm_type, vdc.bfm_period, sig_mask);
		if (rc != 0) {
			LOG_ERROR("cannot connect to %s, Error 0x%x", vdc.bfm_path, rc);
			close_socket(vdc.hsocket);
			vdc.hsocket = 0;
			free(pbuf);
			pbuf = NULL;
		} else {
			for (uint8_t i = 0; i < vdc.mem_ndx; i++) {
				rc = vdebug_mem_open(vdc.hsocket, pbuf, vdc.mem_path[i], i);
				if (rc != 0)
					LOG_WARNING("cannot connect to %s, Error 0x%x", vdc.mem_path[i], rc);
			}

			target_register_timer_callback(vdebug_poll, vdc.poll_min + vdc.poll_max / 2,
										   TARGET_TIMER_TYPE_PERIODIC, &vdc.targ);
			LOG_INFO("vdebug %d connected to %s through %s:%d, polling every",
					 VD_VERSION, vdc.bfm_path, vdc.server_name, vdc.server_port);
		}
	}

	return rc;
}

static int vdebug_quit(void)
{
	int rc;

	target_unregister_timer_callback(vdebug_poll, vdc.targ);
	for (uint8_t i = 0; i < vdc.mem_ndx; i++)
		if (vdc.mem_width[i])
			vdebug_mem_close(vdc.hsocket, pbuf, i);
	rc = vdebug_close(vdc.hsocket, pbuf, VD_BFM_JTAG);
	LOG_INFO("vdebug %d disconnected from %s through %s:%d rc:%d", VD_VERSION,
		vdc.bfm_path, vdc.server_name, vdc.server_port, rc);
	if (vdc.hsocket)
		close_socket(vdc.hsocket);
	free(pbuf);
	pbuf = NULL;

	return ERROR_OK;
}

static int vdebug_reset(int trst, int srst)
{
	uint16_t sig_val = 0xffff;
	uint16_t sig_mask = 0;
	int rc;

	sig_mask |= VD_SIG_RESET;
	if (srst)
		sig_val &= ~VD_SIG_RESET;/* active low */
	if (transport_is_jtag()) {
		sig_mask |= VD_SIG_TRST;
		if (trst)
			sig_val &= ~VD_SIG_TRST; /* active low */
	}

	LOG_INFO("rst trst:%d srst:%d mask:%x val:%x", trst, srst, sig_mask, sig_val);
	rc = vdebug_sig_set(vdc.hsocket, pbuf, sig_mask, sig_val);
	if (rc == 0)
		rc = vdebug_wait(vdc.hsocket, pbuf, 20); /* 20 clock cycles pulse */

	return rc;
}

static int vdebug_tms_seq(const uint8_t *tms, int num, uint8_t f_flush)
{
	LOG_INFO("tms  len:%d tms:%x", num, *(const uint32_t *)tms);

	return vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num, *tms, 0, NULL, 0, 0, NULL, f_flush);
}

static int vdebug_path_move(struct pathmove_command *cmd, uint8_t f_flush)
{
	uint8_t tms[DIV_ROUND_UP(cmd->num_states, 8)];
	LOG_INFO("path num states %d", cmd->num_states);

	memset(tms, 0, DIV_ROUND_UP(cmd->num_states, 8));

	for (uint8_t i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			buf_set_u32(tms, i, 1, 1);
		tap_set_state(cmd->path[i]);
	}

	return vdebug_tms_seq(tms, cmd->num_states, f_flush);
}

static int vdebug_tlr(tap_state_t state, uint8_t f_flush)
{
	int rc = ERROR_OK;
	uint8_t tms_pre;
	uint8_t num_pre;
	uint8_t cur;

	cur = tap_get_state();
	tms_pre = tap_get_tms_path(cur, state);
	num_pre = tap_get_tms_path_len(cur, state);
	LOG_INFO("tlr  from %x to %x", cur, state);
	if (cur != state) {
		rc = vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num_pre, tms_pre, 0, NULL, 0, 0, NULL, f_flush);
		tap_set_state(state);
	}

	return rc;
}

static int vdebug_scan(struct scan_command *cmd, uint8_t f_flush)
{
	int num_bits;
	int rc;
	uint8_t tms_pre, tms_post; /* tms value pre and post shift */
	uint8_t num_pre, num_post; /* num bits pre shift, post shift */
	uint8_t state;
	uint8_t cur;

	cur = tap_get_state();
	state = cmd->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT;
	tms_pre = tap_get_tms_path(cur, state);
	num_pre = tap_get_tms_path_len(cur, state);
	tms_post = tap_get_tms_path(state, cmd->end_state);
	num_post = tap_get_tms_path_len(state, cmd->end_state);
	num_bits = jtag_scan_size(cmd);
	LOG_DEBUG("scan len:%d fields:%d ir/!dr:%d state cur:%x end:%x",
			  num_bits, cmd->num_fields, cmd->ir_scan, cur, cmd->end_state);
	for (int i = 0; i < cmd->num_fields; i++) {
		rc = vdebug_jtag_shift_tap(vdc.hsocket, pbuf, (i == 0 ? num_pre : 0),
			(i == 0 ? tms_pre : 0), cmd->fields[i].num_bits, cmd->fields[i].out_value,
			(i == cmd->num_fields-1 ? num_post : 0), (i == cmd->num_fields-1 ? tms_post : 0),
			cmd->fields[i].in_value, (i == cmd->num_fields-1 ? f_flush : 0));
		if (rc)
			break;
	}

	if (cur != cmd->end_state)
		tap_set_state(cmd->end_state);

	return rc;
}

static int vdebug_runtest(int cycles, tap_state_t state, uint8_t f_flush)
{
	int rc;
	uint8_t tms_pre;
	uint8_t num_pre;
	uint8_t cur;

	cur = tap_get_state();
	tms_pre = tap_get_tms_path(cur, state);
	num_pre = tap_get_tms_path_len(cur, state);
	LOG_DEBUG("idle len:%d state cur:%x end:%x", cycles, cur, state);
	rc = vdebug_jtag_shift_tap(vdc.hsocket, pbuf, num_pre, tms_pre, cycles, NULL, 0, 0, NULL, f_flush);
	if (cur != state)
		tap_set_state(state);

	return rc;
}

static int vdebug_stableclocks(int num, uint8_t f_flush)
{
	LOG_INFO("stab len:%d state cur:%x", num, tap_get_state());

	return vdebug_jtag_shift_tap(vdc.hsocket, pbuf, 0, 0, num, NULL, 0, 0, NULL, f_flush);
}

static int vdebug_sleep(int us)
{
	int rc;

	LOG_INFO("sleep %d us", us);
	rc = vdebug_wait(vdc.hsocket, pbuf, us / 1000);

	return rc;
}

static int vdebug_speed(int speed)
{
	uint32_t divval, clkmax;
	int rc;

	clkmax = VD_SCALE_PSTOMS/(vdc.bfm_period * 2); /* kHz */
	divval = clkmax / speed;
	LOG_INFO("jclk speed:%d kHz set, BFM divider %u", speed, divval);
	rc = vdebug_jtag_clock(vdc.hsocket, pbuf, divval);

	return rc;
}

static int vdebug_khz(int khz, int *jtag_speed)
{
	uint32_t divval, clkmax;

	clkmax = VD_SCALE_PSTOMS/(vdc.bfm_period * 2); /* kHz */
	divval = khz ? clkmax / khz : 1;
	*jtag_speed = clkmax / divval;
	LOG_DEBUG("khz  speed:%d from khz:%d", *jtag_speed, khz);

	return ERROR_OK;
}

static int vdebug_div(int speed, int *khz)
{
	*khz = speed;
	LOG_DEBUG("div  khz:%d from speed:%d", *khz, speed);

	return ERROR_OK;
}

static int vdebug_execute_queue(void)
{
	int rc = ERROR_OK;

	for (struct jtag_command *cmd = jtag_command_queue; rc == ERROR_OK && cmd != NULL; cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RUNTEST:
			rc = vdebug_runtest(cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state, cmd->next == NULL);
			break;
		case JTAG_STABLECLOCKS:
			rc = vdebug_stableclocks(cmd->cmd.stableclocks->num_cycles, cmd->next == NULL);
			break;
		case JTAG_TLR_RESET:
			rc = vdebug_tlr(cmd->cmd.statemove->end_state, cmd->next == NULL);
			break;
		case JTAG_PATHMOVE:
			rc = vdebug_path_move(cmd->cmd.pathmove, cmd->next == NULL);
			break;
		case JTAG_TMS:
			rc = vdebug_tms_seq(cmd->cmd.tms->bits, cmd->cmd.tms->num_bits, cmd->next == NULL);
			break;
		case JTAG_SLEEP:
			rc = vdebug_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			rc = vdebug_scan(cmd->cmd.scan, cmd->next == NULL);
			break;
		default:
			LOG_ERROR("Unknown JTAG command type 0x%x encountered", cmd->type);
			rc = ERROR_FAIL;
		}
	}

	return rc;
}

COMMAND_HANDLER(vdebug_set_server)
{
	char *pchar;
	int rc = ERROR_FAIL;
	if ((CMD_ARGC != 1) || (strchr(CMD_ARGV[0], ':') == NULL))
		LOG_ERROR("server <host>:<port>");
	else {
		pchar = strchr(CMD_ARGV[0], ':');
		*pchar = '\0';
		strncpy(vdc.server_name, CMD_ARGV[0], sizeof(vdc.server_name)-1);
		vdc.server_port = atoi(++pchar);
		rc = ERROR_OK;
	}

	LOG_DEBUG("server: %s port %u", vdc.server_name, vdc.server_port);

	return rc;
}

COMMAND_HANDLER(vdebug_set_bfm)
{
	int rc = ERROR_FAIL;
	char prefix;
	if (CMD_ARGC != 2)
		LOG_ERROR("bfm_path <path> <clk_period[p|n|u]s>");
	else {
		strncpy(vdc.bfm_path, CMD_ARGV[0], sizeof(vdc.bfm_path)-1);
		if (sscanf(CMD_ARGV[1], "%u%cs*", &vdc.bfm_period, &prefix) == 2) {
			switch (prefix) {
			case 'u':
				vdc.bfm_period *= 1000000;
				break;
			case 'n':
				vdc.bfm_period *= 1000;
				break;
			case 'p':
			default:
				break;
			}
			vdc.bfm_type = VD_BFM_JTAG;
			rc = ERROR_OK;
			LOG_DEBUG("bfm_path: %s clk_period %dps", vdc.bfm_path, vdc.bfm_period);
		}
	}

	return rc;
}

COMMAND_HANDLER(vdebug_set_mem)
{
	int rc = ERROR_FAIL;
	if (CMD_ARGC != 3)
		LOG_ERROR("mem_path <path> <base_address> <size>");
	else if (vdc.mem_ndx >= VD_MAX_MEMORIES)
		LOG_ERROR("mem_path declared more than %d allowed times", VD_MAX_MEMORIES);
	else {
		strncpy(vdc.mem_path[vdc.mem_ndx], CMD_ARGV[0], sizeof(vdc.mem_path[vdc.mem_ndx]) - 1);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], vdc.mem_base[vdc.mem_ndx]);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], vdc.mem_size[vdc.mem_ndx]);
		rc = ERROR_OK;
		LOG_DEBUG("mem_path: set %s @ 0x%08x+0x%08x", vdc.mem_path[vdc.mem_ndx],
			vdc.mem_base[vdc.mem_ndx], vdc.mem_size[vdc.mem_ndx]);
		vdc.mem_ndx++;
	}

	return rc;
}

COMMAND_HANDLER(vdebug_set_batching)
{
	int rc = ERROR_FAIL;
	if (CMD_ARGC != 1)
		LOG_ERROR("batching <level>");
	else {
		if (isdigit(CMD_ARGV[0][0]))
			vdc.trans_batch = (CMD_ARGV[0][0] == '0' ? 0 : (CMD_ARGV[0][0] == '1' ? 1 : 2));
		else if (CMD_ARGV[0][0] == 'r')
			vdc.trans_batch = VD_BATCH_WR;
		else if (CMD_ARGV[0][0] == 'w')
			vdc.trans_batch = VD_BATCH_WO;
		else
			vdc.trans_batch = VD_BATCH_NO;
		LOG_DEBUG("batching: set to %u", vdc.trans_batch);
		rc = ERROR_OK;
	}

	return rc;
}

COMMAND_HANDLER(vdebug_set_poll)
{
	int rc = ERROR_FAIL;
	if (CMD_ARGC != 2)
		LOG_ERROR("set_poll <min cycles>> <max cycles>");
	else {
		vdc.poll_min = atoi(CMD_ARGV[0]);
		vdc.poll_max = atoi(CMD_ARGV[1]);
		LOG_DEBUG("poll: set min %u max %u", vdc.poll_min, vdc.poll_max);
		rc = ERROR_OK;
	}

	return rc;
}

COMMAND_HANDLER(vdebug_register_target)
{
	vdc.targ = get_target_by_num(0);
	if ((vdc.targ != NULL) && (vdc.targ->type != NULL))
		LOG_INFO("vdebug_register_target: %s", target_type_name(vdc.targ));
	else
		LOG_ERROR("vdebug_register_target failed, NULL target");
	return ERROR_OK;
}

static const struct command_registration vdebug_command_handlers[] = {
	{
		.name = "server",
		.handler = &vdebug_set_server,
		.mode = COMMAND_CONFIG,
		.help = "set the vdebug server name or address",
		.usage = "server <host:port>",
	},
	{
		.name = "bfm_path",
		.handler = &vdebug_set_bfm,
		.mode = COMMAND_CONFIG,
		.help = "set the vdebug BFM hierarchical path",
		.usage = "bfm_path <path> <clk_period[p|n|u]s>",
	},
	{
		.name = "mem_path",
		.handler = &vdebug_set_mem,
		.mode = COMMAND_ANY,
		.help = "set the design memory for the code load",
		.usage = "mem_path <path> <base_address> <size>",
	},
	{
		.name = "batching",
		.handler = &vdebug_set_batching,
		.mode = COMMAND_CONFIG,
		.help = "set the transaction batching no|wr|rd [0|1|2]",
		.usage = "batching <level>",
	},
	{
		.name = "set_poll",
		.handler = &vdebug_set_poll,
		.mode = COMMAND_CONFIG,
		.help = "set the polling pause, executing hardware cycles between min and max",
		.usage = "set_poll <min cycles> <max cycles>",
	},
	{
		.name = "register_target",
		.handler = &vdebug_register_target,
		.mode = COMMAND_EXEC,
		.help = "Hook up the direct memory access routines and polling to target",
		.usage = "register_target",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration vdebug_command[] = {
	{
		.name = "vdebug",
		.chain = vdebug_command_handlers,
		.mode = COMMAND_ANY,
		.help = "vdebug prefix",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface vdebug_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = vdebug_execute_queue,
};

struct adapter_driver vdebug_adapter_driver = {
	.name = "vdebug",
	.transports = jtag_only,
	.speed = vdebug_speed,
	.khz = vdebug_khz,
	.speed_div = vdebug_div,
	.commands = vdebug_command,
	.init = vdebug_init,
	.quit = vdebug_quit,
	.reset = vdebug_reset,
	.jtag_ops = &vdebug_interface,
};
