/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
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

#include "configuration.h"
#include "log.h"
#include "replacements.h"

static size_t num_config_files;
static char **config_file_names;

static size_t num_script_dirs;
static char **script_search_dirs;

void add_script_search_dir(const char *dir)
{
	num_script_dirs++;
	script_search_dirs = realloc(script_search_dirs, (num_script_dirs + 1) * sizeof(char *));

	script_search_dirs[num_script_dirs-1] = strdup(dir);
	script_search_dirs[num_script_dirs] = NULL;

	LOG_DEBUG("adding %s", dir);
}

void add_config_command(const char *cfg)
{
	num_config_files++;
	config_file_names = realloc(config_file_names, (num_config_files + 1) * sizeof(char *));

	config_file_names[num_config_files-1] = strdup(cfg);
	config_file_names[num_config_files] = NULL;
}

void free_config(void)
{
	while (num_config_files)
		free(config_file_names[--num_config_files]);

	free(config_file_names);
	config_file_names = NULL;

	while (num_script_dirs)
		free(script_search_dirs[--num_script_dirs]);

	free(script_search_dirs);
	script_search_dirs = NULL;
}

/* return full path or NULL according to search rules */
char *find_file(const char *file)
{
	FILE *fp = NULL;
	char **search_dirs = script_search_dirs;
	char *dir;
	char const *mode = "r";
	char *full_path;

	/* Check absolute and relative to current working dir first.
	 * This keeps full_path reporting belowing working. */
	full_path = alloc_printf("%s", file);
	fp = fopen(full_path, mode);

	while (!fp) {
		free(full_path);
		full_path = NULL;
		dir = *search_dirs++;

		if (!dir)
			break;

		full_path = alloc_printf("%s/%s", dir, file);
		fp = fopen(full_path, mode);
	}

	if (fp) {
		fclose(fp);
		LOG_DEBUG("found %s", full_path);
		return full_path;
	}

	free(full_path);

	return NULL;
}

FILE *open_file_from_path(const char *file, const char *mode)
{
	if (mode[0] != 'r')
		return fopen(file, mode);
	else {
		char *full_path = find_file(file);
		if (!full_path)
			return NULL;
		FILE *fp = NULL;
		fp = fopen(full_path, mode);
		free(full_path);
		return fp;
	}
}

int parse_config_file(struct command_context *cmd_ctx)
{
	int retval;
	char **cfg;

	if (!config_file_names) {
		command_run_line(cmd_ctx, "script openocd.cfg");
		return ERROR_OK;
	}

	cfg = config_file_names;

	while (*cfg) {
		retval = command_run_line(cmd_ctx, *cfg);
		if (retval != ERROR_OK)
			return retval;
		cfg++;
	}

	return ERROR_OK;
}

#ifndef _WIN32
#include <pwd.h>
#endif

char *get_home_dir(const char *append_path)
{
	char *home = getenv("HOME");

	if (!home) {

#ifdef _WIN32
		home = getenv("USERPROFILE");

		if (!home) {

			char homepath[MAX_PATH];
			char *drive = getenv("HOMEDRIVE");
			char *path = getenv("HOMEPATH");
			if (drive && path) {
				snprintf(homepath, MAX_PATH, "%s/%s", drive, path);
				home = homepath;
			}
		}
#else
		struct passwd *pwd = getpwuid(getuid());
		if (pwd)
			home = pwd->pw_dir;

#endif
	}

	if (!home)
		return home;

	char *home_path;

	if (append_path)
		home_path = alloc_printf("%s/%s", home, append_path);
	else
		home_path = alloc_printf("%s", home);

	return home_path;
}
