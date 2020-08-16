/***************************************************************************
 *   Copyright (C) 2007-2010 by Øyvind Harboe                              *
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

/* this file contains various functionality useful to standalone systems */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "time_support.h"

#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif
#ifdef HAVE_DIRENT_H
#include <dirent.h>
#endif
#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif
#ifdef HAVE_NET_IF_H
#include <net/if.h>
#endif
#ifdef HAVE_SYS_IOCTL_H
#include <sys/ioctl.h>
#endif
#ifdef HAVE_SYS_STAT_H
#include <sys/stat.h>
#endif
#ifdef HAVE_IFADDRS_H
#include <ifaddrs.h>
#endif
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#endif

/* loads a file and returns a pointer to it in memory. The file contains
 * a 0 byte(sentinel) after len bytes - the length of the file. */
static int load_file(const char *fileName, char **data, size_t *len)
{
	/* ensure returned length is always sane */
	*len = 0;

	FILE *pFile;
	pFile = fopen(fileName, "rb");
	if (pFile == NULL) {
		LOG_ERROR("Can't open %s", fileName);
		return ERROR_FAIL;
	}
	if (fseek(pFile, 0, SEEK_END) != 0) {
		LOG_ERROR("Can't open %s", fileName);
		fclose(pFile);
		return ERROR_FAIL;
	}
	long fsize = ftell(pFile);
	if (fsize == -1) {
		LOG_ERROR("Can't open %s", fileName);
		fclose(pFile);
		return ERROR_FAIL;
	}
	*len = fsize;

	if (fseek(pFile, 0, SEEK_SET) != 0) {
		LOG_ERROR("Can't open %s", fileName);
		fclose(pFile);
		return ERROR_FAIL;
	}
	*data = malloc(*len + 1);
	if (*data == NULL) {
		LOG_ERROR("Can't open %s", fileName);
		fclose(pFile);
		return ERROR_FAIL;
	}

	if (fread(*data, 1, *len, pFile) != *len) {
		fclose(pFile);
		free(*data);
		LOG_ERROR("Can't open %s", fileName);
		return ERROR_FAIL;
	}
	fclose(pFile);

	/* 0-byte after buffer (not included in *len) serves as a sentinel */
	(*data)[*len] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cat_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* NOTE!!! we only have line printing capability so we print the entire file as a single
	 * line. */
	char *data;
	size_t len;

	int retval = load_file(CMD_ARGV[0], &data, &len);
	if (retval == ERROR_OK) {
		command_print(CMD, "%s", data);
		free(data);
	} else
		command_print(CMD, "%s not found", CMD_ARGV[0]);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_trunc_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	FILE *config_file = NULL;
	config_file = fopen(CMD_ARGV[0], "w");
	if (config_file != NULL)
		fclose(config_file);

	return ERROR_OK;
}

#ifdef HAVE_MALLOC_H
COMMAND_HANDLER(handle_meminfo_command)
{
	static int prev;
	struct mallinfo info;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	info = mallinfo();

	if (prev > 0)
		command_print(CMD, "Diff:            %d", prev - info.fordblks);
	prev = info.fordblks;

	command_print(CMD, "Available ram:   %d", info.fordblks);

	return ERROR_OK;
}
#endif

COMMAND_HANDLER(handle_append_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int retval = ERROR_FAIL;
	FILE *config_file = NULL;

	config_file = fopen(CMD_ARGV[0], "a");
	if (config_file != NULL) {
		fseek(config_file, 0, SEEK_END);

		unsigned i;
		for (i = 1; i < CMD_ARGC; i++) {
			if (fwrite(CMD_ARGV[i], 1, strlen(CMD_ARGV[i]),
					config_file) != strlen(CMD_ARGV[i]))
				break;
			if (i != CMD_ARGC - 1) {
				if (fwrite(" ", 1, 1, config_file) != 1)
					break;
			}
		}
		if ((i == CMD_ARGC) && (fwrite("\n", 1, 1, config_file) == 1))
			retval = ERROR_OK;

		fclose(config_file);
	}

	return retval;
}

COMMAND_HANDLER(handle_cp_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* NOTE!!! we only have line printing capability so we print the entire file as a single
	 * line. */
	char *data;
	size_t len;

	int retval = load_file(CMD_ARGV[0], &data, &len);
	if (retval != ERROR_OK)
		return retval;

	FILE *f = fopen(CMD_ARGV[1], "wb");
	if (f == NULL)
		retval = ERROR_COMMAND_SYNTAX_ERROR;

	size_t pos = 0;
	for (;; ) {
		size_t chunk = len - pos;
		static const size_t maxChunk = 512 * 1024;	/* ~1/sec */
		if (chunk > maxChunk)
			chunk = maxChunk;

		if ((retval == ERROR_OK) && (fwrite(data + pos, 1, chunk, f) != chunk))
			retval = ERROR_COMMAND_SYNTAX_ERROR;

		if (retval != ERROR_OK)
			break;

		command_print(CMD, "%zu", len - pos);

		pos += chunk;

		if (pos == len)
			break;
	}

	if (retval == ERROR_OK)
		command_print(CMD, "Copied %s to %s", CMD_ARGV[0], CMD_ARGV[1]);
	else
		command_print(CMD, "copy failed");

	free(data);
	if (f != NULL)
		fclose(f);

	if (retval != ERROR_OK)
		unlink(CMD_ARGV[1]);

	return retval;
}

COMMAND_HANDLER(handle_rm_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bool del = false;
	if (rmdir(CMD_ARGV[0]) == 0)
		del = true;
	else if (unlink(CMD_ARGV[0]) == 0)
		del = true;

	return del ? ERROR_OK : ERROR_FAIL;
}

static int ioutil_Jim_Command_ls(Jim_Interp *interp,
	int argc,
	Jim_Obj * const *argv)
{
	if (argc != 2) {
		Jim_WrongNumArgs(interp, 1, argv, "ls ?dir?");
		return JIM_ERR;
	}

	const char *name = Jim_GetString(argv[1], NULL);

	DIR *dirp = NULL;
	dirp = opendir(name);
	if (dirp == NULL)
		return JIM_ERR;
	Jim_Obj *objPtr = Jim_NewListObj(interp, NULL, 0);

	for (;; ) {
		struct dirent *entry = NULL;
		entry = readdir(dirp);
		if (entry == NULL)
			break;

		if ((strcmp(".", entry->d_name) == 0) || (strcmp("..", entry->d_name) == 0))
			continue;

		Jim_ListAppendElement(interp, objPtr,
			Jim_NewStringObj(interp, entry->d_name, strlen(entry->d_name)));
	}
	closedir(dirp);

	Jim_SetResult(interp, objPtr);

	return JIM_OK;
}

static int ioutil_Jim_Command_peek(Jim_Interp *interp,
	int argc,
	Jim_Obj *const *argv)
{
	if (argc != 2) {
		Jim_WrongNumArgs(interp, 1, argv, "peek ?address?");
		return JIM_ERR;
	}

	long address;
	if (Jim_GetLong(interp, argv[1], &address) != JIM_OK)
		return JIM_ERR;

	int value = *((volatile int *) address);

	Jim_SetResult(interp, Jim_NewIntObj(interp, value));

	return JIM_OK;
}

static int ioutil_Jim_Command_poke(Jim_Interp *interp,
	int argc,
	Jim_Obj *const *argv)
{
	if (argc != 3) {
		Jim_WrongNumArgs(interp, 1, argv, "poke ?address? ?value?");
		return JIM_ERR;
	}

	long address;
	if (Jim_GetLong(interp, argv[1], &address) != JIM_OK)
		return JIM_ERR;
	long value;
	if (Jim_GetLong(interp, argv[2], &value) != JIM_OK)
		return JIM_ERR;

	*((volatile int *) address) = value;

	return JIM_OK;
}

/* not so pretty code to fish out ip number*/
static int ioutil_Jim_Command_ip(Jim_Interp *interp, int argc,
	Jim_Obj *const *argv)
{
#if !defined(__CYGWIN__)
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);

	struct ifaddrs *ifa = NULL, *ifp = NULL;

	if (getifaddrs(&ifp) < 0)
		return JIM_ERR;

	for (ifa = ifp; ifa; ifa = ifa->ifa_next) {
		char ip[200];
		socklen_t salen;

		if (ifa->ifa_addr->sa_family == AF_INET)
			salen = sizeof(struct sockaddr_in);
		else if (ifa->ifa_addr->sa_family == AF_INET6)
			salen = sizeof(struct sockaddr_in6);
		else
			continue;

		if (getnameinfo(ifa->ifa_addr, salen, ip, sizeof(ip), NULL, 0,
				NI_NUMERICHOST) < 0)
			continue;

		Jim_AppendString(interp, tclOutput, ip, strlen(ip));
		break;

	}

	freeifaddrs(ifp);
#else
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "fixme!!!", 0);
	LOG_ERROR("NOT IMPLEMENTED!!!");
#endif
	Jim_SetResult(interp, tclOutput);

	return JIM_OK;
}

#ifdef HAVE_SYS_IOCTL_H
#ifdef SIOCGIFHWADDR
/* not so pretty code to fish out eth0 mac address */
static int ioutil_Jim_Command_mac(Jim_Interp *interp, int argc,
	Jim_Obj *const *argv)
{
	struct ifreq *ifr, *ifend;
	struct ifreq ifreq;
	struct ifconf ifc;
	struct ifreq ifs[5];
	int SockFD;

	SockFD = socket(AF_INET, SOCK_DGRAM, 0);
	if (SockFD < 0)
		return JIM_ERR;

	ifc.ifc_len = sizeof(ifs);
	ifc.ifc_req = ifs;
	if (ioctl(SockFD, SIOCGIFCONF, &ifc) < 0) {
		close(SockFD);
		return JIM_ERR;
	}

	ifend = ifs + (ifc.ifc_len / sizeof(struct ifreq));
	for (ifr = ifc.ifc_req; ifr < ifend; ifr++) {
		/* if (ifr->ifr_addr.sa_family == AF_INET) */
		{
			if (strcmp("eth0", ifr->ifr_name) != 0)
				continue;
			strncpy(ifreq.ifr_name, ifr->ifr_name, sizeof(ifreq.ifr_name) - 1);
			if (ioctl(SockFD, SIOCGIFHWADDR, &ifreq) < 0) {
				close(SockFD);
				return JIM_ERR;
			}

			close(SockFD);

			Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);

			char buffer[256];
			sprintf(buffer, "%02x-%02x-%02x-%02x-%02x-%02x",
				ifreq.ifr_hwaddr.sa_data[0]&0xff,
				ifreq.ifr_hwaddr.sa_data[1]&0xff,
				ifreq.ifr_hwaddr.sa_data[2]&0xff,
				ifreq.ifr_hwaddr.sa_data[3]&0xff,
				ifreq.ifr_hwaddr.sa_data[4]&0xff,
				ifreq.ifr_hwaddr.sa_data[5]&0xff);

			Jim_AppendString(interp, tclOutput, buffer, strlen(buffer));

			Jim_SetResult(interp, tclOutput);

			return JIM_OK;
		}
	}
	close(SockFD);

	return JIM_ERR;

}
#endif
#endif

static const struct command_registration ioutil_command_handlers[] = {
	{
		.name = "cat",
		.handler = handle_cat_command,
		.mode = COMMAND_ANY,
		.help = "display text file content",
		.usage = "file_name",
	},
	{
		.name = "trunc",
		.handler = handle_trunc_command,
		.mode = COMMAND_ANY,
		.help = "truncate a file to zero length",
		.usage = "file_name",
	},
	{
		.name = "cp",
		.handler = handle_cp_command,
		.mode = COMMAND_ANY,
		.help = "copy a file",
		.usage = "src_file_name dst_file_name",
	},
	{
		.name = "append_file",
		.handler = handle_append_command,
		.mode = COMMAND_ANY,
		.help = "append a variable number of strings to a file",
		.usage = "file_name [<string1>, [<string2>, ...]]",
	},
#ifdef HAVE_MALLOC_H
	{
		.name = "meminfo",
		.handler = handle_meminfo_command,
		.mode = COMMAND_ANY,
		.help = "display free heap space",
		.usage = "",
	},
#endif
	{
		.name = "rm",
		.mode = COMMAND_ANY,
		.handler = handle_rm_command,
		.help = "remove a directory or file",
		.usage = "file_name",
	},

	/*
	 * Peek and poke are security holes -- they manipulate
	 * server-internal addresses.
	 */

	/* jim handlers */
	{
		.name = "peek",
		.mode = COMMAND_ANY,
		.jim_handler = ioutil_Jim_Command_peek,
		.help = "peek at a memory address",
		.usage = "address",
	},
	{
		.name = "poke",
		.mode = COMMAND_ANY,
		.jim_handler = ioutil_Jim_Command_poke,
		.help = "poke at a memory address",
		.usage = "address value",
	},
	{
		.name = "ls",
		.mode = COMMAND_ANY,
		.jim_handler = ioutil_Jim_Command_ls,
		.help = "show a listing of files",
		.usage = "dirname",
	},
#ifdef HAVE_SYS_IOCTL_H
#ifdef SIOCGIFHWADDR
	{
		.name = "mac",
		.mode = COMMAND_ANY,
		.jim_handler = ioutil_Jim_Command_mac,
		.help = "show MAC address",
	},
#endif
#endif
	{
		.name = "ip",
		.jim_handler = ioutil_Jim_Command_ip,
		.mode = COMMAND_ANY,
		.help = "show IP address",
	},
	COMMAND_REGISTRATION_DONE
};

int ioutil_init(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, ioutil_command_handlers);
}
