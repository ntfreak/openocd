//#----------------------------------------------------------------------------
//# Copyright 2020-2021 Cadence Design Systems, Inc.
//#
//# Redistribution and use in source and binary forms, with or without modification,
//# are permitted provided that the following conditions are met:
//# 1. Redistributions of source code must retain the above copyright notice,
//# this list of conditions and the following disclaimer.
//# 2. Redistributions in binary form must reproduce the above copyright notice,
//# this list of conditions and the following disclaimer in the documentation
//# and/or other materials provided with the distribution.
//#----------------------------------------------------------------------------
//# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//#----------------------------------------------------------------------------
//# Revisions   :
//# 41 10.12.20 : based on vd_client and vd_test
//# 42 16.02.21 : shift_tap fix
//#----------------------------------------------------------------------------

/*!
 * @file
 *
 * @brief the virtual debug interface provides a connection between a sw debugger
 * and the simulated, emulated core over a soft connection, implemented by DPI
 * It implements an interface and JTAG, DAP and AMBA transports
 *
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#define SOCKET int
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
#include "helper/log.h"

#define VD_VERSION 42
#define VD_BUILD "08.03.21"
#define VD_BUFFER_LEN 4024
#define VD_CHEADER_LEN 24
#define VD_SHEADER_LEN 16

#define VD_MAX_MEMORIES 3

/**
 * @brief List of transactor types
 */
typedef enum {
  VD_BFM_JTDP  = 0x0001,  /**< transactor DAP JTAG DP */
  VD_BFM_SWDP  = 0x0002,  /**< transactor DAP SWD DP */
  VD_BFM_AHB   = 0x0003,  /**< transactor AMBA AHB */
  VD_BFM_APB   = 0x0004,  /**< transactor AMBA APB */
  VD_BFM_AXI   = 0x0005,  /**< transactor AMBA AXI */
  VD_BFM_JTAG  = 0x0006,  /**< transactor serial JTAG */
  VD_BFM_SWD   = 0x0007,  /**< transactor serial SWD */
} vd_bfm_et;

/**
 * @brief List of signals that can be read or written by the debugger
 */
typedef enum {
  VD_SIG_TCK   = 0x0001,  /**< JTAG clock; tclk */
  VD_SIG_TDI   = 0x0002,  /**< JTAG TDI;   tdi */
  VD_SIG_TMS   = 0x0004,  /**< JTAG TMS;   tms */
  VD_SIG_RESET = 0x0008,  /**< DUT reset;  rst */
  VD_SIG_TRST  = 0x0010,  /**< JTAG Reset; trstn */
  VD_SIG_TDO   = 0x0020,  /**< JTAG TDO;   tdo */
  VD_SIG_POWER = 0x0100,  /**< BFM power;  bfm_up */
  VD_SIG_TCKDIV= 0x0200,  /**< JTAG clock divider; tclkdiv */
  VD_SIG_BUF   = 0x1000,  /**< memory buffer; mem */
} vd_sig_et;

/**
 * @brief List of errors
 */
typedef enum {
  VD_ERR_NONE      = 0x0000,  /**< no error */
  VD_ERR_NOT_IMPL  = 0x0100,  /**< feature not implemented */
  VD_ERR_USAGE     = 0x0101,  /**< incorrect usage */
  VD_ERR_PARAM     = 0x0102,  /**< incorrect parameter */
  VD_ERR_CONFIG    = 0x0107,  /**< incorrect configuration */
  VD_ERR_NO_MEMORY = 0x0104,  /**< out of memory */
  VD_ERR_SHM_OPEN  = 0x010a,  /**< cannot open shared memory */
  VD_ERR_SHM_MAP   = 0x010b,  /**< cannot map shared memory */
  VD_ERR_SOC_OPEN  = 0x011a,  /**< cannot open socket */
  VD_ERR_SOC_OPT   = 0x011b,  /**< cannot set socket option */
  VD_ERR_SOC_ADDR  = 0x011c,  /**< cannot resolve host address */
  VD_ERR_SOC_CONN  = 0x011d,  /**< cannot connect to host */
  VD_ERR_SOC_SEND  = 0x011e,  /**< error sending data on socket */
  VD_ERR_SOC_RECV  = 0x011f,  /**< error receiving data from socket */
  VD_ERR_LOCKED    = 0x0202,  /**< device locked */
  VD_ERR_NOT_RUN   = 0x0204,  /**< transactor not running */
  VD_ERR_NOT_OPEN  = 0x0205,  /**< transactor not open/connected */
  VD_ERR_LICENSE   = 0x0206,  /**< cannot check out the license */
  VD_ERR_VERSION   = 0x0207,  /**< transactor version mismatch */
  VD_ERR_TIME_OUT  = 0x0301,  /**< time out, waiting */
  VD_ERR_NO_POWER  = 0x0302,  /**< power out error */
  VD_ERR_BUS_ERROR = 0x0304,  /**< bus protocol error, like pslverr */
  VD_ERR_NO_ACCESS = 0x0306,  /**< no access to an object */
  VD_ERR_INV_HANDLE= 0x0307,  /**< invalid object handle */
  VD_ERR_INV_SCOPE = 0x0308,  /**< invalid scope */
} vd_err_et;

typedef struct
{
  struct
  {                          // VD_CHEADER_LEN written by client
    uint8_t cmd;             // 000;
    uint8_t type;            // 001;
    uint16_t waddr;          // 002;
    uint16_t wbytes;         // 004;
    uint16_t rbytes;         // 006;
    uint16_t wwords;         // 008;
    uint16_t rwords;         // 00a;
    uint32_t rwdata;         // 00c;
    uint32_t offset;         // 010;
    uint16_t offseth;        // 014;
    uint16_t wid;            // 016;
  };
  union
  {                          // 018;
    uint8_t wd8[VD_BUFFER_LEN];
    uint32_t wd32[VD_BUFFER_LEN/4];
    uint64_t wd64[VD_BUFFER_LEN/8];
  };
  struct
  {                          // VD_SHEADER_LEN written by server
    uint16_t rid;            // fd0: 
    uint16_t awords;         // fd2:
    int32_t  status;         // fd4;
    uint64_t duttime;        // fd8;
  };
  union
  {                          // fe0:
    uint8_t rd8[VD_BUFFER_LEN];
    uint32_t rd32[VD_BUFFER_LEN/4];
    uint64_t rd64[VD_BUFFER_LEN/8];
  };
  uint32_t state;            // 1f98;
  uint32_t count;            // 1f9c;
  uint8_t dummy[96];         // 1fa0; 48+40B+8B;
} vd_shm_st;

static const char* const vdebug_transports[] = { "jtag", NULL };

static FILE* debug_log = NULL;
static uint16_t debug = 0;
static uint8_t debug_out = 0;
static uint8_t trans_batch = 1;
static uint8_t trans_first = 0;
static uint8_t trans_last = 0;
static uint8_t mem_ndx = 0;
static uint8_t buf_width;
static uint8_t addr_bits;
static uint8_t bfm_type;
static uint16_t sig_read;
static uint16_t sig_write;
static uint32_t bfm_period;
static uint32_t mem_base[VD_MAX_MEMORIES];
static uint32_t mem_size[VD_MAX_MEMORIES];
static uint32_t mem_width[VD_MAX_MEMORIES];
static uint32_t mem_depth[VD_MAX_MEMORIES];
static uint32_t server_port;
static uint32_t pollcycles = 10000;
static uint32_t pollmin;
static uint32_t pollmax;
static int hsocket;
static char server_name[32];
static char bfm_path[128];
static char mem_path[VD_MAX_MEMORIES][128];
static vd_shm_st* pbuf = NULL;

static struct target* ptarg = NULL;
static int (*targ_poll)(struct target* ptar) = NULL;
static int (*targ_write_memory)(struct target *target, target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer) = NULL;

static void debug_open(void)
{
  char msg[32];
  char *p = getenv( "VD_DEBUG" );
  debug = (p ? strtol( p, NULL, 0 ) : 0);
  if( debug )
  {
    if( (p = getenv( "VD_LOG" )) == NULL )
    {
      strcpy( msg, "vd_client.log" );
      debug_log = fopen( msg, "w" );
    }
    else
      debug_log = fopen( p, "w" );
    if( debug > 0x100 )
    {
      debug_out = debug >> 8;
      debug &= 0x00ff;
    }
    sprintf( msg, "OpenOCD vdebug client %d", VD_VERSION );
    if( debug_log )
    {
      fputs( msg, debug_log ); fputc( '\n', debug_log );
    }
  }
}

static uint32_t debug_msg( uint32_t er_code, char* format, ...)
{
  char msg[128];
  long long unsigned duttime = (pbuf ? pbuf->duttime : 0);
  if (format)
  {
    va_list ap;
    snprintf( msg, 15, "%10lluns: ", duttime );
    va_start(ap,format);     // explicitly terminating string, Windows bug
    vsnprintf(msg+14, sizeof(msg)-15, format, ap); msg[127]='\0';
    if( er_code )
      LOG_ERROR("%s: vdebug code 0x%x", msg, er_code);
    else if( debug_out )
      LOG_DEBUG("%s", msg);
    va_end(ap);
    if( debug && debug_log )
    {
      fputs( msg, debug_log ); fputc( '\n', debug_log );
    }
  }
  return er_code;
}

static int socket_error(void)
{
#ifdef _WIN32
  return WSAGetLastError();
#else
  return errno;
#endif
}

static int socket_close(SOCKET hsock)
{
#ifdef _WIN32
  closesocket(hsock);
  WSACleanup();
#else
  close(hsock);
#endif
  return 0;
}

static SOCKET socket_open(char* server_addr, uint32_t port)
{
  SOCKET hsock;
  int rc = 0;
  uint32_t buflen = sizeof(vd_shm_st); // size of the send and rcv buffer
  struct addrinfo *ainfo = NULL;
  struct addrinfo ahint = { 0, AF_INET, SOCK_STREAM, 0, 0, NULL, NULL, NULL };

#ifdef _WIN32
  WSADATA ver;
  if ((rc = WSAStartup(MAKEWORD(2, 2), &ver)) != 0)
    ;
  else if ((hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) == INVALID_SOCKET)
    rc = debug_msg( VD_ERR_SOC_OPEN, "socket_open: cannot open socket, error %d", socket_error() );
#else
  uint32_t rcvwat = VD_SHEADER_LEN;    // size of the rcv header, as rcv min watermark
  if ((hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) < 0)
    rc = debug_msg( VD_ERR_SOC_OPEN, "socket_open: cannot open socket, error %d", socket_error() );
  else if (setsockopt(hsock, SOL_SOCKET, SO_RCVLOWAT, &rcvwat, sizeof(rcvwat)) < 0)
    rc = errno;
#endif
  else if (setsockopt(hsock, SOL_SOCKET, SO_SNDBUF, (const char*)&buflen, sizeof(buflen)) < 0)
    rc = socket_error();
  else if (setsockopt(hsock, SOL_SOCKET, SO_RCVBUF, (const char*)&buflen, sizeof(buflen)) < 0)
    rc = socket_error();
  if (rc)
    rc = debug_msg( VD_ERR_SOC_OPT, "socket_open: cannot set socket option, error %d", rc );
  else if ((rc = getaddrinfo(server_addr, NULL, &ahint, &ainfo)) != 0)
    rc = debug_msg( VD_ERR_SOC_ADDR, "socket_open: cannot resolve address %s, error %d", server_addr, rc );
  else
  {
    ((struct sockaddr_in*)(ainfo->ai_addr))->sin_port = htons(port);
    if (connect(hsock, ainfo->ai_addr, sizeof(struct sockaddr)) < 0)
      rc = debug_msg( VD_ERR_SOC_CONN, "socket_open: cannot connect to %s:%d, error %d", server_addr, port, socket_error() );
  }
  if (rc)
  {
    socket_close(hsock);
    hsock = 0;
  }
  if (ainfo)
    freeaddrinfo(ainfo);
  return hsock;
}

static int socket_receive(SOCKET hsock, vd_shm_st* pmem)
{
  int rc;
  uint16_t dreceived = 0;
  uint16_t offset = (uint8_t*)&pmem->rid - &pmem->cmd;
  uint16_t to_receive = VD_SHEADER_LEN + pmem->rbytes;
  char *pb = (char*)pmem;

  do
  {
    if ((rc = recv(hsock, pb + offset, to_receive, 0)) <= 0)
      break;
    else
    {              // the data can come in pieces
      to_receive -= rc;
      offset += rc;
    }
    if (debug > 2)
      debug_msg( VD_ERR_NONE, "socket_receive: received %u, to receive %u", rc, to_receive );
    dreceived += rc;
  } while (rc > 0 && to_receive);
  if (rc <= 0)
    debug_msg( VD_ERR_SOC_RECV, "socket_receive: recv failed, error %d", socket_error() );
  else
    rc = dreceived;
  return rc;
}

static int socket_send(SOCKET hsock, vd_shm_st* pmem)
{
  int rc;
  if( (rc = send(hsock, (const char*)&pmem->cmd, VD_CHEADER_LEN + pmem->wbytes, 0)) <= 0)
    debug_msg( VD_ERR_SOC_SEND, "socket_send: send failed, error %d", socket_error() );
  else if (debug > 2)
    debug_msg( VD_ERR_NONE, "socket_send: sent %u, to send %u", rc, 0 );
  return rc;
}

static uint32_t wait_server( SOCKET hsock, vd_shm_st* pmem )
{
  int st, rd;
  if( !hsock )
    st = VD_ERR_SOC_OPEN;
  else if( (st = socket_send(hsock, pmem)) <= 0 )
    st = VD_ERR_SOC_SEND;
  else if( (rd = socket_receive(hsock, pmem)) <= 0 )
    st = VD_ERR_SOC_RECV;
  else
  {
    if( debug > 3 )
      debug_msg( 0, "wait_server: cmd %02hx done, sent %d, rcvd %d, status %d time %llu", pmem->cmd, st, rd, pmem->status, pmem->duttime );
    st = pmem->status;
  }
  return st;
}

static int vdebug_open( SOCKET hsock, vd_shm_st* pm, const char* path, uint32_t type, uint32_t period_ps, uint32_t sig_mask )
{
  int rc;
  
  if( pm )
  {
    pm->cmd = 0x01;
    pm->wid = (uint16_t)VD_VERSION;
    pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
    if( (rc = wait_server( hsock, pm )) != 0 ) // communication problem
      debug_msg( rc, "vdebug_open: Error 0x%x connecting to server", rc );
    else if( pm->rid < pm->wid )       // communication OK, but version wrong
    {
      debug_msg( VD_ERR_VERSION, "vdebug_open: server version %d too old for the client %d", pm->rid, pm->wid );
      pm->cmd = 0x02;                  // let server close the connection
      wait_server( hsock, pm );
      rc = 0x207;
    }
    else
    {
      pm->cmd = 0x04;
      pm->type = (uint8_t)type;
      pm->rwdata = sig_mask | VD_SIG_BUF | (VD_SIG_BUF << 16);
      pm->wbytes = (uint16_t)strlen(path)+1; pm->rbytes = 12;
      pm->wid = 0;             // reset wid for transaction ID
      pm->wwords = pm->rwords = 0;
      memcpy( pm->wd8, path, pm->wbytes+1 );
      rc = wait_server( hsock, pm );
      bfm_type = type;
      sig_read = (uint16_t)(pm->rwdata >> 16);    // signal read mask
      sig_write = (uint16_t)pm->rwdata;   // signal write mask
      bfm_period = period_ps;
      buf_width = pm->rd32[0]/8;// access width in bytes
      addr_bits = pm->rd32[2];
    }
    if( rc )
      debug_msg( rc, "vdebug_open: Error 0x%x connecting to BFM %s", rc, path ); 
    else if( debug )
      debug_msg( VD_ERR_NONE, "vdebug_open: %s type %0x, period %dps, buffer %dx%dB signals r%04xw%04x",
      path, bfm_type, bfm_period, VD_BUFFER_LEN/buf_width, buf_width, sig_read, sig_write );
  }
  else
    rc = VD_ERR_NOT_OPEN;
  return rc;
}

static int vdebug_close( SOCKET hsock, vd_shm_st* pm, uint32_t type )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm )
  {
    pm->cmd = 0x05;
    pm->type = (uint8_t)type;
    pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
    rc = wait_server( hsock, pm );
    pm->cmd = 0x02;
    pm->wid = (uint16_t)VD_VERSION;
    pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
    wait_server( hsock, pm );
    if( debug )
      debug_msg( VD_ERR_NONE, "vdebug_close: type %0x", type );
  }
  return rc;
}

static int vdebug_wait( SOCKET hsock, vd_shm_st* pm, uint32_t cycles )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm && cycles )
  {
    pm->cmd = 0x09;
    pm->wbytes = 0;
    pm->rbytes = 0;
    pm->rwdata = cycles;
    rc = wait_server( hsock, pm );
    if( debug )
      debug_msg( VD_ERR_NONE, "vdebug_wait: %d cycles", cycles );
  }
  return rc;
}

static int vdebug_sig_set( SOCKET hsock, vd_shm_st* pm, uint32_t write_mask, uint32_t value )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm )
  {
    pm->cmd = 0x0a;
    pm->wbytes = 0;
    pm->rbytes = 0;
    pm->rwdata = (write_mask << 16) | (value & 0xffff);
    rc = wait_server( hsock, pm );
    if( rc ) 
      debug_msg( rc, "vdebug_sig_set: Error 0x%x setting signals %04x", rc, write_mask );
    else if( debug )
      debug_msg( VD_ERR_NONE, "vdebug_sig_set: setting signals %04x to %04x", write_mask, value );
  }
  return rc;
}

static int vdebug_jtag_clock( SOCKET hsock, vd_shm_st* pm, uint32_t value )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm )
  {
    pm->cmd = 0x0f;
    pm->wbytes = 0;
    pm->rbytes = 0;
    pm->rwdata = value;
    rc = wait_server( hsock, pm );
    if( rc ) 
      debug_msg( rc, "vdebug_jtag_clock: Error 0x%x setting jtag_clock", rc );
    else if( debug )
      debug_msg( VD_ERR_NONE, "vdebug_jtag_clock: setting jtag clock divider to %d", value );
  }
  return rc;
}

static int vdebug_jtag_shift_tap( SOCKET hsock, vd_shm_st* pm, uint8_t num_pre, const uint8_t tms_pre, uint32_t num, const uint8_t* tdi, uint8_t num_post, const uint8_t tms_post, uint8_t* tdo, uint8_t f_last )
{
  const uint32_t tobits = 8;
  uint8_t ftdi, ftms;
  uint16_t req, rreq, i,j;
  uint16_t bytes, hwords, anum, words, rwords, waddr;
  uint8_t* data;
  int rc = 0;
  
  if( pm )
  {
    pm->cmd = 0x1a;
    trans_last = f_last || (trans_batch == 0) || (trans_batch == 1 && tdo);
    if( trans_first )
      waddr = 0;             // reset buffer offset
    else
      waddr = pm->offseth;   // continue from the previous transaction
    if( num_post )           // actual number of bits to shift
      anum = num + num_pre + num_post - 1;
    else
      anum = num + num_pre;
    hwords = (anum+4*buf_width-1)/(4*buf_width); // in 4B TDI/TMS words
    words = (hwords+1)/2;    // in 8B TDO words to read
    bytes = (num+7)/8;       // data only portion in bytes
                             // buffer overflow check and flush
    if( waddr + 2 + 2*hwords + 16 > VD_BUFFER_LEN/4 )
      trans_last = 1;        // force flush within 64B of buffer end
    else if( waddr + 2 + 2*hwords > VD_BUFFER_LEN/4 )
    {                        // this req does not fit, discard it
      rc = debug_msg( VD_ERR_NO_MEMORY, "vdebug_jtag_shift_tap: %04x Error too many bits, discarded L:%02d O:%05x @%04x", pm->wid, anum, ((trans_first << 14)|(trans_last << 15)), waddr );
    }
    if( !rc && anum )        // support for calls with num=0 as flush
    {
      pm->wd32[waddr++] = (tdo ? 0xc0000000: 0x40000000) + (num_pre << 27) + (num_post << 24) + anum;
      pm->wd32[waddr++] = hwords + (tdo ? (words << 16): 0);
      pm->wid++;
      pm->wd8[4*waddr] = (tdi ? (tdi[0] << num_pre) : 0);
      pm->wd8[4*waddr+4] = tms_pre;    // init with tms_pre
      if( num+num_pre <= 8 )           // and tms_post for num <=4
        pm->wd8[4*waddr+4] |= (tms_post << (num+num_pre-1));
      for( i=1,j=4*waddr; i<bytes; i++ )  // copy the tdi and tms data
      {
        if( i == bytes-1 && num+num_pre <= bytes*tobits )
          pm->wd8[j+i+4] = tms_post << ((num+num_pre-1) % 8);
        else
          pm->wd8[j+i+4] = 0x0;// placing 4 bytes of TMS bits into high word
        if( !tdi )             // placing 4 bytes of TDI bits into low word
          pm->wd8[j+i] = 0x0;
        else
          pm->wd8[j+i] = (tdi[i] << num_pre) | (tdi[i-1] >> (8-num_pre));
        if( i % 4 == 3 )
          j += 4;
      }
      if( tdi )
      {
        if( num+num_pre > bytes*tobits )// in case 1 additional byte needed for TDI
          pm->wd8[j+i] = (tdi[i-1] >> (8-num_pre)); // put last TDI bits there
      }
      if( num+num_pre <= bytes*tobits )// in case no or 1 additional byte needed 
        pm->wd8[j+i+4] = tms_post >> (8-(num+num_pre-1) % 8); // may need to add higher part
                                       // in case exactly 1 additional byte needed
      else if( num+num_pre > bytes*tobits && anum <= (bytes+1)*tobits )
        pm->wd8[j+i+4] = tms_post << ((num+num_pre-1) % 8); // add whole tms_post
      else                             // in case 2 additional bytes, tms_post split
      {
        pm->wd8[j+i+4] = tms_post << ((num+num_pre-1) % 8);// add lower part of tms_post
        if( i % 4 == 3 )               // next byte is in the next 32b word
          pm->wd8[j+i+4+5] = tms_post >> (8-(num+num_pre-1) % 8); // and higher part
        else                           // next byte is in the same 32b word
          pm->wd8[j+i+4+1] = tms_post >> (8-(num+num_pre-1) % 8); // and higher part
      }
      if( tdo )
      {
        pm->rwords += words;           // keep track of the words to read
        pm->rd64[VD_BUFFER_LEN/8-(++pm->offset)] = (uintptr_t)tdo;
      }
      pm->wwords = waddr/2 + hwords;   // payload size *2 to include both TDI and TMS data
      pm->waddr++;
    }

    if( !waddr )                       // no action
      ;
    else if( !trans_last )             // buffered request
      pm->offseth = waddr + hwords*2;  // offset for next transaction, must be even
    else                               // execute batch of requests
    {
      req = rreq = waddr = rwords = 0;   // beginning of request
      pm->wbytes = pm->wwords*8;
      pm->rbytes = pm->rwords*8;
      if( (rc = wait_server( hsock, pm )) != 0)
        debug_msg( rc, "vdebug_jtag_shift_tap: Error 0x%x executing transaction", rc );
      else do
      {                                  // loop over requests to read data and print out
        hwords = pm->wd32[waddr+1];      // reconstrcut data this request
        anum = pm->wd32[waddr] & 0xffff;
        num_pre = (pm->wd32[waddr] >> 27) & 0x7;
        num_post = (pm->wd32[waddr] >> 24) & 0x7;
        if( num_post )
          num = anum - num_pre - num_post + 1;
        else
          num = anum - num_pre;
        words = (hwords+1)/2;
        bytes = (num+7)/tobits;
        trans_last = (req+1 < pm->waddr ? 0 : 1); 
        trans_first = (waddr ? 0 : 1);
        if( pm->wd32[waddr] & 0x80000000 )
        {
          data = (uint8_t*)(uintptr_t)pm->rd64[VD_BUFFER_LEN/8-1-rreq++];
          for( j=0; j<bytes; j++ )
          {
            data[j] = (pm->rd8[rwords*8+j] >> num_pre) | (pm->rd8[rwords*8+j+1] << (tobits-num_pre));
            if( debug > 4 )
              debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x D0[%02x]:%02x", pm->wid-pm->waddr+req, j, data[j] );
          }
          rwords += words;               // read data offset
        }
        else
          data = NULL;
        if( debug )
        {
          ftdi = (pm->wd8[(waddr+2)*4] >> num_pre) | (pm->wd8[(waddr+2)*4+1] << (8-num_pre));
          ftms = (pm->wd8[(waddr+2)*4+4] >> num_pre) | (pm->wd8[(waddr+2)*4+4+1] << (8-num_pre));
          debug_msg( VD_ERR_NONE, "vd_jtag_shift_tap: %04x L:%02d O:%05x @%03x DI:%02x MS:%02x DO:%02x", pm->wid-pm->waddr+req, num, ((trans_first << 14)|(trans_last << 15)), waddr, ftdi, ftms, (data ? data[0] : 0xdd) );
        }
        waddr += 2+hwords*2;             // next request beginning
      } while( ++req < pm->waddr );
      pm->offseth = 0;       // reset buffer write address
      pm->offset = 0;
      pm->rwords = 0;
      pm->waddr = 0;
    }    
    trans_first = trans_last;          // flush forces trans_first flag
  }
  else
    rc = VD_ERR_NOT_OPEN;
  return rc;
}

static int vdebug_mem_open( SOCKET hsock, vd_shm_st* pm, const char* path, uint8_t ndx )
{
  int rc = VD_ERR_NOT_OPEN;
  
  if( pm )
  {
    pm->cmd = 0x21;
    pm->wbytes = (uint16_t)strlen( path )+1;
    pm->rbytes = 8;
    pm->wwords = pm->rwords = 0;
    memcpy( pm->wd8, path, pm->wbytes+1 );
    rc = wait_server( hsock, pm );
    if( rc ) 
      debug_msg( rc, "vdebug_mem_open: Error 0x%x opening memory %s", rc, path );
    else
    {
      mem_width[ndx] = pm->rd32[0]/8;     // memory width in bytes
      mem_depth[ndx] = pm->rd32[1];       // memory depth in words
      if( debug )
        debug_msg( VD_ERR_NONE, "vdebug_mem_open: %s memory %xx%dB, buffer %dx%dB",
        path, mem_depth[ndx], mem_width[ndx], VD_BUFFER_LEN/mem_width[ndx], mem_width[ndx] );
    }
  }
  return rc;
}

static void vdebug_mem_close(  SOCKET hsock, vd_shm_st* pm, uint8_t ndx )
{
  if( pm )
  {
    pm->cmd = 0x22;
    pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
    wait_server( hsock, pm );
    if( debug )
      debug_msg( VD_ERR_NONE, "vdebug_mem_close: %s", mem_path[ndx] );
  }
}

static int vdebug_mem_write( SOCKET hsock, vd_shm_st* pm, uint8_t ndx, uint64_t addr, uint32_t num, const uint8_t* data )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm )
  {
    pm->cmd = 0x23;
    pm->wbytes = num;
    pm->wwords = (num+mem_width[ndx]-1)/mem_width[ndx];
    pm->rbytes = pm->rwords = 0;
#ifndef _M_IX86
    pm->offset = (uint32_t)(addr/mem_width[ndx]);
    pm->offseth = (uint16_t)(addr/mem_width[ndx] >> 32);
#else
    pm->offset = ((uint32_t)(addr))/mem_width[ndx]; // 32 bit calculation for Win32
    pm->offseth = 0;
#endif    
    memcpy( pm->wd8, data, num );
    rc = wait_server( hsock, pm );
    if( rc )
      debug_msg( rc, "vdebug_mem_write: Error 0x%x writing %d bytes at %llx", rc, num, addr );
    else if( debug )
      debug_msg( VD_ERR_NONE, "vdebug_mem_write: A:%08llx L:%d D:%08x", addr, num, pm->wd32[0] );
    pm->offset = 0;
    pm->offseth = 0;
  }
  return rc;
}

static int vdebug_poll( struct target* pt )
{
  int rc;
  struct timeval ts, te;
  uint32_t cmdtime;

  assert( pt != NULL );
  gettimeofday( &ts, NULL );
  if( targ_poll )
    rc = targ_poll( pt );
  if( pt->state == TARGET_RUNNING )
    pollcycles = pollmax;
  else
    pollcycles = pollmin;
  vdebug_wait( hsocket, pbuf, pollcycles );
  gettimeofday( &ts, NULL );
  cmdtime = (uint32_t)((te.tv_sec - ts.tv_sec) * 1000000 + te.tv_usec - ts.tv_usec);
  LOG_DEBUG("poll state:%u cycles:%u executed in %uus", pt->state, pollcycles, cmdtime );
  return rc;
}

static int vdebug_write_memory( struct target* pt, target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer)
{
  int rc;
  int ndx;
  uint32_t offset;
  uint32_t written;
  uint32_t towrite;
  assert( pt != NULL );
  for( ndx = 0; ndx < VD_MAX_MEMORIES; ndx++ )
    if( address >= mem_base[ndx] && address+size*count <= mem_base[ndx]+mem_size[ndx] )
      break;                 // hit one of the declared memories
  if( ndx < VD_MAX_MEMORIES && size*count > 4 )
  {
    if( ndx != mem_ndx && mem_width[mem_ndx] )
    {                        // need to switch memory
      vdebug_mem_close( hsocket, pbuf, mem_ndx );
      mem_width[mem_ndx] = 0;
    }
    mem_ndx = ndx;
    if( mem_width[ndx] == 0 )
      rc = vdebug_mem_open( hsocket, pbuf, mem_path[ndx], ndx );
    else
      rc = 0;
    offset = address-mem_base[ndx];
    LOG_INFO("vdebug_mem_write: i:%d a:0x%08x n:%d rc:%d", ndx, mem_base[ndx]+offset, count*size, rc );
    written = 0;
    while( !rc && written < count*size )
    {
      towrite = (count*size - written < VD_BUFFER_LEN ? count*size - written : VD_BUFFER_LEN);
      rc = vdebug_mem_write( hsocket, pbuf, ndx, offset, towrite, buffer+written );
      offset += towrite;
      written += towrite;
    }
  }
  else if( targ_write_memory )
    rc = targ_write_memory( pt, address, size, count, buffer );
  LOG_DEBUG("write_memory: " TARGET_ADDR_FMT " n:%d w:%d", address, count, size );
  return rc;
}

static int vdebug_init(void)
{
  uint32_t type, sig_mask;
  int rc;

  debug_open();
  for( rc = 0; rc < VD_MAX_MEMORIES; rc++ )
    mem_width[rc] = 0;
  pollmin = 1000; pollmax = 5000; pollcycles = pollmax;
  type = VD_BFM_JTAG;
  sig_mask = VD_SIG_RESET | VD_SIG_TRST | VD_SIG_TCKDIV;
  rc = ERROR_OK;
  if( (hsocket = socket_open( server_name, server_port )) <= 0 )
    rc = VD_ERR_SOC_OPEN;
  else if( (pbuf = (vd_shm_st*)calloc( 1, sizeof( vd_shm_st ))) == NULL )
  {
    socket_close( hsocket );
    hsocket = 0;
    LOG_ERROR("cannot allocate %lu bytes", sizeof( vd_shm_st ) );
  }
  else if( (rc = vdebug_open( hsocket, pbuf, bfm_path, type, bfm_period, sig_mask)) != 0 )
    ;
  else
    LOG_INFO("vdebug %d %s connected to %s through %s:%d", VD_VERSION, VD_BUILD, bfm_path, server_name, server_port);
  trans_first = 1;
  return rc;
}

static int vdebug_quit(void)
{
  int rc;
  
  targ_write_memory = NULL;  // target is already destroyed at this point
  targ_poll = NULL;
  for( rc = 0; rc < VD_MAX_MEMORIES; rc++ )
    if( mem_width[rc] )
      vdebug_mem_close( hsocket, pbuf, rc );
  rc = vdebug_close( hsocket, pbuf, VD_BFM_JTAG );
  if( hsocket )
    socket_close( hsocket );
  if( pbuf )
    free( pbuf );
  if( debug_log )
    fclose( debug_log );
  pbuf = NULL;
  hsocket = -1;
  debug_log = NULL;
  LOG_INFO("vdebug %d disconnected from %s through %s:%d", VD_VERSION, bfm_path, server_name, server_port);
  return rc;
}

static int vdebug_reset(int trst, int srst)
{
  uint16_t sig_val = 0xffff;
  uint16_t sig_mask = 0;
  int rc;

  sig_mask |= VD_SIG_RESET;
  if( srst )
    sig_val &= ~VD_SIG_RESET;// active low
  if( transport_is_jtag() )
  {
    sig_mask |= VD_SIG_TRST;
    if( trst )
      sig_val &= ~VD_SIG_TRST; // active low  
  }
  LOG_INFO("rst trst:%d srst:%d mask:%x val:%x", trst, srst, sig_mask, sig_val);
  if( (rc = vdebug_sig_set( hsocket, pbuf, sig_mask, sig_val )) != 0 )
    ;
  else
    rc = vdebug_wait( hsocket, pbuf, 20 );

  return rc;
}

static int vdebug_tms_seq(const uint8_t *tms, int num, uint8_t f_flush)
{
  LOG_INFO("tms  len:%d tms:%x", num, *(const uint32_t*)tms);
  return vdebug_jtag_shift_tap( hsocket, pbuf, num, *tms, 0, NULL, 0, 0, NULL, f_flush );
}

static int vdebug_path_move(struct pathmove_command *cmd, uint8_t f_flush)
{
  uint8_t tms[DIV_ROUND_UP(cmd->num_states, 8)];
  LOG_INFO("path num states %d", cmd->num_states );

  memset(tms, 0, DIV_ROUND_UP(cmd->num_states, 8));

  for (int i = 0; i < cmd->num_states; i++) {
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
  tms_pre = tap_get_tms_path( cur, state );
  num_pre = tap_get_tms_path_len( cur, state );
  LOG_INFO("tlr  from %x to %x", cur, state );
  if (cur != state)
  {
    rc = vdebug_jtag_shift_tap( hsocket, pbuf, num_pre, tms_pre, 0, NULL, 0, 0, NULL, f_flush );
    tap_set_state(state);
  }
  return rc;
}

static int vdebug_scan(struct scan_command *cmd, uint8_t f_flush)
{
  int num_bits;
  int i, rc;
  uint8_t tms_pre, tms_post; // tms value pre and post shift
  uint8_t num_pre, num_post; // num bits pre shift, post shift
  uint8_t state;
  uint8_t cur;

  cur = tap_get_state();
  state = cmd->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT;
  tms_pre = tap_get_tms_path( cur, state );
  num_pre = tap_get_tms_path_len( cur, state );
  tms_post = tap_get_tms_path( state, cmd->end_state );
  num_post = tap_get_tms_path_len( state, cmd->end_state );
  num_bits = jtag_scan_size( cmd );
  LOG_DEBUG("scan len:%d fields:%d ir/!dr:%d state cur:%x end:%x", num_bits, cmd->num_fields, cmd->ir_scan, cur, cmd->end_state );
  for( i=0, rc=0; rc == 0 && i < cmd->num_fields; i++ )
    rc = vdebug_jtag_shift_tap( hsocket, pbuf, (i==0 ? num_pre : 0), (i==0 ? tms_pre : 0), cmd->fields[i].num_bits, cmd->fields[i].out_value, (i==cmd->num_fields-1 ? num_post : 0), (i==cmd->num_fields-1 ? tms_post : 0), cmd->fields[i].in_value, (i==cmd->num_fields-1 ? f_flush : 0) );
  if( cur != cmd->end_state )
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
  tms_pre = tap_get_tms_path( cur, state );
  num_pre = tap_get_tms_path_len( cur, state );
  LOG_DEBUG("idle len:%d state cur:%x end:%x", cycles, cur, state );
  rc = vdebug_jtag_shift_tap( hsocket, pbuf, num_pre, tms_pre, cycles, NULL, 0, 0, NULL, f_flush );
  if( cur != state )
    tap_set_state( state );
  return rc;
}

static int vdebug_stableclocks(int num, uint8_t f_flush)
{
  LOG_INFO("stab len:%d state cur:%x", num, tap_get_state() );
  return vdebug_jtag_shift_tap( hsocket, pbuf, 0, 0, num, NULL, 0, 0, NULL, f_flush );
}

static int vdebug_sleep(int us)
{
  int rc;
  
  LOG_INFO("sleep %d us", us );
  rc = vdebug_wait( hsocket, pbuf, us/1000 );
  return rc;
}

static int vdebug_speed(int speed)
{
  uint32_t divval, clkmax;
  int rc;
  
  clkmax = 1000000000/(bfm_period * 2); // kHz
  divval = clkmax/speed;
  LOG_INFO("jclk speed:%d kHz set, BFM divider %u", speed, divval );
  rc = vdebug_jtag_clock( hsocket, pbuf, divval );
  return rc;
}

static int vdebug_khz(int khz, int* jtag_speed)
{
  uint32_t divval, clkmax;
  
  clkmax = 1000000000/(bfm_period * 2); // kHz
  divval = khz ? clkmax/khz : 1;
  *jtag_speed = clkmax/divval;
  LOG_DEBUG("khz  speed:%d from khz:%d", *jtag_speed, khz );
  return ERROR_OK;
}

static int vdebug_div(int speed, int* khz)
{
  *khz = speed;
  LOG_DEBUG("div  khz:%d from speed:%d", *khz, speed );
  return ERROR_OK;
}

static int vdebug_execute_queue(void)
{
  struct jtag_command *cmd;
  int retval = ERROR_OK;

  for (cmd = jtag_command_queue; retval == ERROR_OK && cmd != NULL; cmd = cmd->next)
  {
    switch (cmd->type)
    {
      case JTAG_RESET:
        retval = vdebug_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
        break;
      case JTAG_RUNTEST:
        retval = vdebug_runtest(cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state, cmd->next == NULL);
        break;
      case JTAG_STABLECLOCKS:
        retval = vdebug_stableclocks(cmd->cmd.stableclocks->num_cycles, cmd->next == NULL);
        break;
      case JTAG_TLR_RESET:
        retval = vdebug_tlr(cmd->cmd.statemove->end_state, cmd->next == NULL);
        break;
      case JTAG_PATHMOVE:
        retval = vdebug_path_move(cmd->cmd.pathmove, cmd->next == NULL);
        break;
      case JTAG_TMS:
        retval = vdebug_tms_seq(cmd->cmd.tms->bits, cmd->cmd.tms->num_bits, cmd->next == NULL);
        break;
      case JTAG_SLEEP:
        retval = vdebug_sleep(cmd->cmd.sleep->us);
        break;
      case JTAG_SCAN:
        retval = vdebug_scan(cmd->cmd.scan, cmd->next == NULL);
        break;
    }
  }
  return retval;
}

static int vdebug_config_trace(bool enabled, enum tpiu_pin_protocol pin_protocol,
                uint32_t port_size, unsigned int *trace_freq, unsigned int traceclkin_freq, uint16_t *prescaler )
{
  *trace_freq = 0;
  LOG_DEBUG("config_trace en:%d prot:%u size:%u freq:%u scale:%u", enabled, pin_protocol, port_size, traceclkin_freq, *prescaler);
  return ERROR_OK;
}

static int vdebug_poll_trace( uint8_t* buf, size_t* size )
{
  *size = 0;
  LOG_DEBUG("poll_trace  ");
  return vdebug_wait( hsocket, pbuf, pollcycles );
}

COMMAND_HANDLER(vdebug_set_server)
{
  char* pchar;
  int rc = ERROR_FAIL;
  if (CMD_ARGC == 0)
  {
    strcpy( server_name, "localhost" );
    server_port = 8192;    
  }
  else if( (pchar = strchr( CMD_ARGV[0], ':' )) != NULL )
  {                          // server:port
    *pchar = '\0';
    strncpy( server_name, CMD_ARGV[0], sizeof(server_name)-1 );
    server_port = atoi( ++pchar );
    rc = ERROR_OK;
  }
  else
  {
    strncpy( server_name, CMD_ARGV[0], sizeof(server_name)-1 );
    server_port = 8192;
    rc = ERROR_OK;
  }
  LOG_DEBUG("vdebug server: %s port %u", server_name, server_port);
  return rc;
}

COMMAND_HANDLER(vdebug_set_bfm)
{
  int rc = ERROR_FAIL;
  char prefix;
  if (CMD_ARGC != 2)
    LOG_ERROR("vdebug bfm_path <path> <clk_period[p|n|u]s>");
  else
  {
    strncpy( bfm_path, CMD_ARGV[0], sizeof(bfm_path)-1 );
    if( sscanf( CMD_ARGV[1], "%u%cs*", &bfm_period, &prefix ) == 2 )
    {
      switch( prefix )
      {   
        case 'u': bfm_period *= 1000000;
          break;
        case 'n': bfm_period *= 1000;
          break;
        case 'p':
        default:
          break;
      }
      rc = ERROR_OK;
      LOG_DEBUG("vdebug bfm_path: %s clk_period %dps", bfm_path, bfm_period);
    }
  }
  return rc;
}

COMMAND_HANDLER(vdebug_set_mem)
{
  int rc = ERROR_FAIL;
  if (CMD_ARGC != 3)
    LOG_ERROR("mem_path <path> <base_address> <size>");
  else if( mem_ndx >= VD_MAX_MEMORIES )
    LOG_ERROR("mem_path declared more than %d allowed times", VD_MAX_MEMORIES);
  else
  {
    strncpy( mem_path[mem_ndx], CMD_ARGV[0], sizeof(mem_path[mem_ndx])-1 );
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], mem_base[mem_ndx]);
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], mem_size[mem_ndx]);
    rc = ERROR_OK;
    LOG_DEBUG("vdebug mem_path: set %s @ 0x%08x+0x%08x", mem_path[mem_ndx], mem_base[mem_ndx], mem_size[mem_ndx]);
    mem_ndx++;
  }
  return rc;
}

COMMAND_HANDLER(vdebug_set_batching)
{
  int rc = ERROR_FAIL;

  if (CMD_ARGC != 1)
    LOG_ERROR("transaction_batching <level>");
  else 
  {
    if( isdigit( CMD_ARGV[0][0] ) )
      trans_batch = (CMD_ARGV[0][0] == '0' ? 0 : (CMD_ARGV[0][0] == '1' ? 1: 2 ));
    else if( CMD_ARGV[0][0] == 'r' )
      trans_batch = 2;
    else if( CMD_ARGV[0][0] == 'w' )
      trans_batch = 1;
    else
      trans_batch = 0;
    LOG_DEBUG("transaction_batching: set to %u", trans_batch);
    rc = ERROR_OK;
  }
  return rc;
}

COMMAND_HANDLER(vdebug_register_poll)
{
  int rc = ERROR_FAIL;

  if (CMD_ARGC != 2)
    LOG_ERROR("register_target_poll <min cycles>> <max cycles>");
  else
  {
    pollmin = atoi( CMD_ARGV[0] );
    pollmax = atoi( CMD_ARGV[1] );
    if( ((ptarg = get_target_by_num(0)) != NULL) && ptarg->type != NULL )
    {
      targ_poll = ptarg->type->poll;
      ptarg->type->poll = &vdebug_poll;
      LOG_INFO("register_target_poll: set min %u max %u cycles", pollmin, pollmax);
    }
    rc = ERROR_OK;
  }
  return rc;
}

COMMAND_HANDLER(vdebug_register_dma)
{
  int rc = ERROR_FAIL;

  if( ((ptarg = get_target_by_num(0)) != NULL) && ptarg->type != NULL )
  {
    targ_write_memory = ptarg->type->write_memory;
    ptarg->type->write_memory = &vdebug_write_memory;
    LOG_INFO("register_target_dma: %u target memories registered for direct access", mem_ndx);
    rc = ERROR_OK;
  }
  return rc;
}

static const struct command_registration vdebug_command_handlers[] = 
{
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
    .name = "transaction_batching",
    .handler = &vdebug_set_batching,
    .mode = COMMAND_CONFIG,
    .help = "set the transaction batching no|wr|rd [0|1|2]",
    .usage = "transaction_batching <level>",
  },
  {
    .name = "register_target_poll",
    .handler = &vdebug_register_poll,
    .mode = COMMAND_EXEC,
    .help = "set the polling pause, executing hardware cycles between min and max",
    .usage = "register_target_poll <min cycles> <max cycles>",
  },
  {
    .name = "register_target_dma",
    .handler = &vdebug_register_dma,
    .mode = COMMAND_EXEC,
    .help = "Hook up the direct memory access routines to the memories specified by mem_path",
    .usage = "register_target_dma",
  },
  COMMAND_REGISTRATION_DONE
};

static struct jtag_interface vdebug_interface = 
{
  .supported = DEBUG_CAP_TMS_SEQ,
  .execute_queue = vdebug_execute_queue,
};

struct adapter_driver vdebug_adapter_driver = 
{
  .name = "vdebug",
  .transports = vdebug_transports,
  .speed = vdebug_speed,
  .khz = vdebug_khz,
  .speed_div = vdebug_div,
  .commands = vdebug_command_handlers,
  .init = vdebug_init,
  .quit = vdebug_quit,
  .config_trace = vdebug_config_trace,
  .poll_trace = vdebug_poll_trace,
  .jtag_ops = &vdebug_interface,
};
