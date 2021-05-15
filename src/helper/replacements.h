/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
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

#ifndef OPENOCD_HELPER_REPLACEMENTS_H
#define OPENOCD_HELPER_REPLACEMENTS_H

#include <stdint.h>
#include <helper/system.h>

/* MIN,MAX macros */
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/* for systems that do not support ENOTSUP
 * win32 being one of them */
#ifndef ENOTSUP
#define ENOTSUP 134		/* Not supported */
#endif

/* for systems that do not support O_BINARY
 * linux being one of them */
#ifndef O_BINARY
#define O_BINARY 0
#endif

#ifndef HAVE_SYS_TIME_H

#ifndef _TIMEVAL_DEFINED
#define _TIMEVAL_DEFINED

struct timeval {
	long tv_sec;
	long tv_usec;
};

#endif	/* _TIMEVAL_DEFINED */

#endif

/* gettimeofday() */
#ifndef HAVE_GETTIMEOFDAY

#ifdef _WIN32
struct timezone {
	int tz_minuteswest;
	int tz_dsttime;
};
#endif
struct timezone;

int gettimeofday(struct timeval *tv, struct timezone *tz);

#endif

#ifndef IN_REPLACEMENTS_C
/**** clear_malloc & fill_malloc ****/
void *clear_malloc(size_t size);
void *fill_malloc(size_t size);
#endif

/*
 * Now you have 3 ways for the malloc function:
 *
 * 1. Do not change anything, use the original malloc
 *
 * 2. Use the clear_malloc function instead of the original malloc.
 *    In this case you must use the following define:
 *    #define malloc((_a)) clear_malloc((_a))
 *
 * 3. Use the fill_malloc function instead of the original malloc.
 *    In this case you must use the following define:
 *    #define malloc((_a)) fill_malloc((_a))
 *
 * We have figured out that there could exist some malloc problems
 * where variables are using without to be initialise. To find this
 * places, use the fill_malloc function. With this function we want
 * to initialize memory to some known bad state. This is quite easily
 * spotted in the debugger and will trap to an invalid address.
 *
 * clear_malloc can be used if you want to set not initialise
 * variable to 0.
 *
 * If you do not want to change the malloc function, to not use one of
 * the following macros. Which is the default way.
 */

/* #define malloc(_a) clear_malloc(_a)
 * #define malloc(_a) fill_malloc(_a) */

/* GNU extensions to the C library that may be missing on some systems */
#ifndef HAVE_STRNDUP
char *strndup(const char *s, size_t n);
#endif	/* HAVE_STRNDUP */

#ifndef HAVE_STRNLEN
size_t strnlen(const char *s, size_t maxlen);
#endif	/* HAVE_STRNLEN */

#ifndef HAVE_USLEEP
#ifdef _WIN32
static inline unsigned usleep(unsigned int usecs)
{
	Sleep((usecs/1000));
	return 0;
}
#else
#error no usleep defined for your platform
#endif
#endif	/* HAVE_USLEEP */

/* Windows specific */
#ifdef _WIN32

#include <windows.h>
#include <time.h>

/* Windows does not declare sockaddr_un */
#define UNIX_PATH_LEN 108
struct sockaddr_un {
	uint16_t sun_family;
	char sun_path[UNIX_PATH_LEN];
};

/* win32 systems do not support ETIMEDOUT */

#ifndef ETIMEDOUT
#define ETIMEDOUT WSAETIMEDOUT
#endif

#if IS_MINGW == 1
static inline unsigned char inb(unsigned short int port)
{
	unsigned char _v;
	__asm__ __volatile__ ("inb %w1,%0" : "=a" (_v) : "Nd" (port));
	return _v;
}

static inline void outb(unsigned char value, unsigned short int port)
{
	__asm__ __volatile__ ("outb %b0,%w1" : : "a" (value), "Nd" (port));
}

/* mingw does not have ffs, so use gcc builtin types */
#define ffs __builtin_ffs

#endif	/* IS_MINGW */

int win_select(int max_fd, fd_set *rfds, fd_set *wfds, fd_set *efds, struct timeval *tv);

#endif	/* _WIN32 */

/* generic socket functions for Windows and Posix */
static inline int write_socket(int handle, const void *buffer, unsigned int count)
{
#ifdef _WIN32
	return send(handle, buffer, count, 0);
#else
	return write(handle, buffer, count);
#endif
}

static inline int read_socket(int handle, void *buffer, unsigned int count)
{
#ifdef _WIN32
	return recv(handle, buffer, count, 0);
#else
	return read(handle, buffer, count);
#endif
}

static inline int close_socket(int sock)
{
#ifdef _WIN32
	return closesocket(sock);
#else
	return close(sock);
#endif
}

static inline void socket_block(int fd)
{
#ifdef _WIN32
	unsigned long nonblock = 0;
	ioctlsocket(fd, FIONBIO, &nonblock);
#else
	int oldopts = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, oldopts & ~O_NONBLOCK);
#endif
}

static inline void socket_nonblock(int fd)
{
#ifdef _WIN32
	unsigned long nonblock = 1;
	ioctlsocket(fd, FIONBIO, &nonblock);
#else
	int oldopts = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, oldopts | O_NONBLOCK);
#endif
}

static inline int socket_select(int max_fd,
	fd_set *rfds,
	fd_set *wfds,
	fd_set *efds,
	struct timeval *tv)
{
#ifdef _WIN32
	return win_select(max_fd, rfds, wfds, efds, tv);
#else
	return select(max_fd, rfds, wfds, efds, tv);
#endif
}

#ifndef HAVE_ELF_H

typedef uint32_t Elf32_Addr;
typedef uint16_t Elf32_Half;
typedef uint32_t Elf32_Off;
typedef uint32_t Elf32_Word;
typedef uint32_t Elf32_Size;

#define EI_NIDENT   16

typedef struct {
	unsigned char e_ident[EI_NIDENT];	/* Magic number and other info */
	Elf32_Half e_type;			/* Object file type */
	Elf32_Half e_machine;			/* Architecture */
	Elf32_Word e_version;			/* Object file version */
	Elf32_Addr e_entry;			/* Entry point virtual address */
	Elf32_Off e_phoff;			/* Program header table file offset */
	Elf32_Off e_shoff;			/* Section header table file offset */
	Elf32_Word e_flags;			/* Processor-specific flags */
	Elf32_Half e_ehsize;			/* ELF header size in bytes */
	Elf32_Half e_phentsize;		/* Program header table entry size */
	Elf32_Half e_phnum;			/* Program header table entry count */
	Elf32_Half e_shentsize;		/* Section header table entry size */
	Elf32_Half e_shnum;			/* Section header table entry count */
	Elf32_Half e_shstrndx;			/* Section header string table index */
} Elf32_Ehdr;

#define ELFMAG			"\177ELF"
#define SELFMAG			4

#define EI_CLASS		4		/* File class byte index */
#define ELFCLASS32		1		/* 32-bit objects */
#define ELFCLASS64		2		/* 64-bit objects */

#define EI_DATA			5		/* Data encoding byte index */
#define ELFDATA2LSB		1		/* 2's complement, little endian */
#define ELFDATA2MSB		2		/* 2's complement, big endian */

typedef struct {
	Elf32_Word p_type;		/* Segment type */
	Elf32_Off p_offset;		/* Segment file offset */
	Elf32_Addr p_vaddr;		/* Segment virtual address */
	Elf32_Addr p_paddr;		/* Segment physical address */
	Elf32_Size p_filesz;	/* Segment size in file */
	Elf32_Size p_memsz;		/* Segment size in memory */
	Elf32_Word p_flags;		/* Segment flags */
	Elf32_Size p_align;		/* Segment alignment */
} Elf32_Phdr;

#define PT_LOAD			1		/* Loadable program segment */

#endif	/* HAVE_ELF_H */

#ifndef HAVE_ELF64

typedef uint64_t Elf64_Addr;
typedef uint16_t Elf64_Half;
typedef uint64_t Elf64_Off;
typedef uint32_t Elf64_Word;
typedef uint64_t Elf64_Xword;

typedef struct {
	unsigned char e_ident[EI_NIDENT];	/* Magic number and other info */
	Elf64_Half e_type;			/* Object file type */
	Elf64_Half e_machine;		/* Architecture */
	Elf64_Word e_version;		/* Object file version */
	Elf64_Addr e_entry;			/* Entry point virtual address */
	Elf64_Off e_phoff;			/* Program header table file offset */
	Elf64_Off e_shoff;			/* Section header table file offset */
	Elf64_Word e_flags;			/* Processor-specific flags */
	Elf64_Half e_ehsize;		/* ELF header size in bytes */
	Elf64_Half e_phentsize;		/* Program header table entry size */
	Elf64_Half e_phnum;			/* Program header table entry count */
	Elf64_Half e_shentsize;		/* Section header table entry size */
	Elf64_Half e_shnum;			/* Section header table entry count */
	Elf64_Half e_shstrndx;		/* Section header string table index */
} Elf64_Ehdr;

typedef struct {
	Elf64_Word p_type;		/* Segment type */
	Elf64_Word p_flags;		/* Segment flags */
	Elf64_Off p_offset;		/* Segment file offset */
	Elf64_Addr p_vaddr;		/* Segment virtual address */
	Elf64_Addr p_paddr;		/* Segment physical address */
	Elf64_Xword p_filesz;	/* Segment size in file */
	Elf64_Xword p_memsz;	/* Segment size in memory */
	Elf64_Xword p_align;	/* Segment alignment */
} Elf64_Phdr;

#endif /* HAVE_ELF64 */

#endif /* OPENOCD_HELPER_REPLACEMENTS_H */
