/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008, Duane Ellis                                       *
 *   openocd@duaneeellis.com                                               *
 *                                                                         *
 *   part of this file is taken from libcli (libcli.sourceforge.net)       *
 *   Copyright (C) David Parrish (david@dparrish.com)                      *
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

/* see Embedded-HOWTO.txt in Jim Tcl project hosted on BerliOS*/
#define JIM_EMBEDDED

/* @todo the inclusion of target.h here is a layering violation */
#include <jtag/jtag.h>
#include <target/target.h>
#include "command.h"
#include "configuration.h"
#include "log.h"
#include "time_support.h"
#include "jim-eventloop.h"

/* nice short description of source file */
#define __THIS__FILE__ "command.c"

struct log_capture_state {
	Jim_Interp *interp;
	Jim_Obj *output;
};

static int unregister_command(struct command_context *context,
	const char *cmd_prefix, const char *name);
static int jim_command_dispatch(Jim_Interp *interp, int argc, Jim_Obj * const *argv);
static int help_add_command(struct command_context *cmd_ctx,
	const char *cmd_name, const char *help_text, const char *usage_text);
static int help_del_command(struct command_context *cmd_ctx, const char *cmd_name);

/* set of functions to wrap jimtcl internal data */
static inline bool jimcmd_is_proc(Jim_Cmd *cmd)
{
	return cmd->isproc;
}

bool jimcmd_is_oocd_command(Jim_Cmd *cmd)
{
	return !cmd->isproc && cmd->u.native.cmdProc == jim_command_dispatch;
}

void *jimcmd_privdata(Jim_Cmd *cmd)
{
	return cmd->isproc ? NULL : cmd->u.native.privData;
}

static void tcl_output(void *privData, const char *file, unsigned line,
	const char *function, const char *string)
{
	struct log_capture_state *state = privData;
	Jim_AppendString(state->interp, state->output, string, strlen(string));
}

static struct log_capture_state *command_log_capture_start(Jim_Interp *interp)
{
	/* capture log output and return it. A garbage collect can
	 * happen, so we need a reference count to this object */
	Jim_Obj *jim_output = Jim_NewStringObj(interp, "", 0);
	if (!jim_output)
		return NULL;

	Jim_IncrRefCount(jim_output);

	struct log_capture_state *state = malloc(sizeof(*state));
	if (!state) {
		LOG_ERROR("Out of memory");
		Jim_DecrRefCount(interp, jim_output);
		return NULL;
	}

	state->interp = interp;
	state->output = jim_output;

	log_add_callback(tcl_output, state);

	return state;
}

/* Classic openocd commands provide progress output which we
 * will capture and return as a Tcl return value.
 *
 * However, if a non-openocd command has been invoked, then it
 * makes sense to return the tcl return value from that command.
 *
 * The tcl return value is empty for openocd commands that provide
 * progress output.
 *
 * Therefore we set the tcl return value only if we actually
 * captured output.
 */
static void command_log_capture_finish(struct log_capture_state *state)
{
	if (!state)
		return;

	log_remove_callback(tcl_output, state);

	int length;
	Jim_GetString(state->output, &length);

	if (length > 0)
		Jim_SetResult(state->interp, state->output);
	else {
		/* No output captured, use tcl return value (which could
		 * be empty too). */
	}
	Jim_DecrRefCount(state->interp, state->output);

	free(state);
}

/*
 * FIXME: workaround for memory leak in jimtcl 0.80
 * Jim API Jim_CreateCommand() converts the command name in a Jim object and
 * does not free the object. Fixed for jimtcl 0.81 by e4416cf86f0b
 * Use the internal jimtcl API Jim_CreateCommandObj, not exported by jim.h,
 * and override the bugged API through preprocessor's macro.
 * This workaround works only when jimtcl is compiled as OpenOCD submodule.
 * It's broken on macOS, so it's currently restricted on Linux only.
 * If jimtcl is linked-in from a precompiled library, either static or dynamic,
 * the symbol Jim_CreateCommandObj is not exported and the build will use the
 * bugged API.
 * To be removed when OpenOCD will switch to jimtcl 0.81
 */
#if JIM_VERSION == 80 && defined __linux__
static int workaround_createcommand(Jim_Interp *interp, const char *cmdName,
	Jim_CmdProc *cmdProc, void *privData, Jim_DelCmdProc *delProc);
int Jim_CreateCommandObj(Jim_Interp *interp, Jim_Obj *cmdNameObj,
	Jim_CmdProc *cmdProc, void *privData, Jim_DelCmdProc *delProc)
__attribute__((weak, alias("workaround_createcommand")));
static int workaround_createcommand(Jim_Interp *interp, const char *cmdName,
	Jim_CmdProc *cmdProc, void *privData, Jim_DelCmdProc *delProc)
{
	if ((void *)Jim_CreateCommandObj == (void *)workaround_createcommand)
		return Jim_CreateCommand(interp, cmdName, cmdProc, privData, delProc);

	Jim_Obj *cmd_name = Jim_NewStringObj(interp, cmdName, -1);
	Jim_IncrRefCount(cmd_name);
	int retval = Jim_CreateCommandObj(interp, cmd_name, cmdProc, privData, delProc);
	Jim_DecrRefCount(interp, cmd_name);
	return retval;
}
#define Jim_CreateCommand workaround_createcommand
#endif /* JIM_VERSION == 80 && defined __linux__*/
/* FIXME: end of workaround for memory leak in jimtcl 0.80 */

static int command_retval_set(Jim_Interp *interp, int retval)
{
	int *return_retval = Jim_GetAssocData(interp, "retval");
	if (return_retval)
		*return_retval = retval;

	return (retval == ERROR_OK) ? JIM_OK : retval;
}

extern struct command_context *global_cmd_ctx;

/* dump a single line to the log for the command.
 * Do nothing in case we are not at debug level 3 */
static void script_debug(Jim_Interp *interp, unsigned int argc, Jim_Obj * const *argv)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	char *dbg = alloc_printf("command -");
	for (unsigned i = 0; i < argc; i++) {
		int len;
		const char *w = Jim_GetString(argv[i], &len);
		char *t = alloc_printf("%s %s", dbg, w);
		free(dbg);
		dbg = t;
	}
	LOG_DEBUG("%s", dbg);
	free(dbg);
}

static void script_command_args_free(char **words, unsigned nwords)
{
	for (unsigned i = 0; i < nwords; i++)
		free(words[i]);
	free(words);
}

static char **script_command_args_alloc(
	unsigned argc, Jim_Obj * const *argv, unsigned *nwords)
{
	char **words = malloc(argc * sizeof(char *));
	if (!words)
		return NULL;

	unsigned i;
	for (i = 0; i < argc; i++) {
		int len;
		const char *w = Jim_GetString(argv[i], &len);
		words[i] = strdup(w);
		if (words[i] == NULL) {
			script_command_args_free(words, i);
			return NULL;
		}
	}
	*nwords = i;
	return words;
}

struct command_context *current_command_context(Jim_Interp *interp)
{
	/* grab the command context from the associated data */
	struct command_context *cmd_ctx = Jim_GetAssocData(interp, "context");
	if (!cmd_ctx) {
		/* Tcl can invoke commands directly instead of via command_run_line(). This would
		 * happen when the Jim Tcl interpreter is provided by eCos or if we are running
		 * commands in a startup script.
		 *
		 * A telnet or gdb server would provide a non-default command context to
		 * handle piping of error output, have a separate current target, etc.
		 */
		cmd_ctx = global_cmd_ctx;
	}
	return cmd_ctx;
}

/**
 * Find a openocd command from fullname.
 * @returns Returns the named command if it is registred in interp.
 * Returns NULL otherwise.
 */
static struct command *command_find_from_name(Jim_Interp *interp, const char *name)
{
	if (!name)
		return NULL;

	Jim_Obj *jim_name = Jim_NewStringObj(interp, name, -1);
	Jim_IncrRefCount(jim_name);
	Jim_Cmd *cmd = Jim_GetCommand(interp, jim_name, JIM_NONE);
	Jim_DecrRefCount(interp, jim_name);
	if (!cmd || jimcmd_is_proc(cmd) || !jimcmd_is_oocd_command(cmd))
		return NULL;

	return jimcmd_privdata(cmd);
}

static struct command *command_new(struct command_context *cmd_ctx,
	const char *full_name, const struct command_registration *cr)
{
	assert(cr->name);

	/*
	 * If it is a non-jim command with no .usage specified,
	 * log an error.
	 *
	 * strlen(.usage) == 0 means that the command takes no
	 * arguments.
	*/
	if (!cr->jim_handler && !cr->usage)
		LOG_ERROR("BUG: command '%s' does not have the "
			"'.usage' field filled out",
			full_name);

	struct command *c = calloc(1, sizeof(struct command));
	if (!c)
		return NULL;

	c->name = strdup(cr->name);
	if (!c->name) {
		free(c);
		return NULL;
	}

	c->handler = cr->handler;
	c->jim_handler = cr->jim_handler;
	c->mode = cr->mode;

	if (cr->help || cr->usage)
		help_add_command(cmd_ctx, full_name, cr->help, cr->usage);

	return c;
}

static void command_free(struct Jim_Interp *interp, void *priv)
{
	struct command *c = priv;

	free(c->name);
	free(c);
}

static struct command *register_command(struct command_context *context,
	const char *cmd_prefix, const struct command_registration *cr)
{
	char *full_name;

	if (!context || !cr->name)
		return NULL;

	if (cmd_prefix)
		full_name = alloc_printf("%s %s", cmd_prefix, cr->name);
	else
		full_name = strdup(cr->name);
	if (!full_name)
		return NULL;

	struct command *c = command_find_from_name(context->interp, full_name);
	if (c) {
		/* TODO: originally we treated attempting to register a cmd twice as an error
		 * Sometimes we need this behaviour, such as with flash banks.
		 * http://www.mail-archive.com/openocd-development@lists.berlios.de/msg11152.html */
		LOG_DEBUG("command '%s' is already registered", full_name);
		free(full_name);
		return c;
	}

	c = command_new(context, full_name, cr);
	if (!c) {
		free(full_name);
		return NULL;
	}

	if (false) /* too noisy with debug_level 3 */
		LOG_DEBUG("registering '%s'...", full_name);
	int retval = Jim_CreateCommand(context->interp, full_name,
				jim_command_dispatch, c, command_free);
	if (retval != JIM_OK) {
		command_run_linef(context, "del_help_text {%s}", full_name);
		command_run_linef(context, "del_usage_text {%s}", full_name);
		free(c);
		free(full_name);
		return NULL;
	}

	free(full_name);
	return c;
}

int __register_commands(struct command_context *cmd_ctx, const char *cmd_prefix,
	const struct command_registration *cmds, void *data,
	struct target *override_target)
{
	int retval = ERROR_OK;
	unsigned i;
	for (i = 0; cmds[i].name || cmds[i].chain; i++) {
		const struct command_registration *cr = cmds + i;

		struct command *c = NULL;
		if (cr->name) {
			c = register_command(cmd_ctx, cmd_prefix, cr);
			if (!c) {
				retval = ERROR_FAIL;
				break;
			}
			c->jim_handler_data = data;
			c->jim_override_target = override_target;
		}
		if (cr->chain) {
			if (cr->name) {
				if (cmd_prefix) {
					char *new_prefix = alloc_printf("%s %s", cmd_prefix, cr->name);
					if (!new_prefix) {
						retval = ERROR_FAIL;
						break;
					}
					retval = __register_commands(cmd_ctx, new_prefix, cr->chain, data, override_target);
					free(new_prefix);
				} else {
					retval = __register_commands(cmd_ctx, cr->name, cr->chain, data, override_target);
				}
			} else {
				retval = __register_commands(cmd_ctx, cmd_prefix, cr->chain, data, override_target);
			}
			if (retval != ERROR_OK)
				break;
		}
	}
	if (retval != ERROR_OK) {
		for (unsigned j = 0; j < i; j++)
			unregister_command(cmd_ctx, cmd_prefix, cmds[j].name);
	}
	return retval;
}

static __attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 2, 3)))
int unregister_commands_match(struct command_context *cmd_ctx, const char *format, ...)
{
	Jim_Interp *interp = cmd_ctx->interp;
	va_list ap;

	va_start(ap, format);
	char *query = alloc_vprintf(format, ap);
	va_end(ap);
	if (!query)
		return ERROR_FAIL;

	char *query_cmd = alloc_printf("info commands {%s}", query);
	free(query);
	if (!query_cmd)
		return ERROR_FAIL;

	int retval = Jim_EvalSource(interp, __THIS__FILE__, __LINE__, query_cmd);
	free(query_cmd);
	if (retval != JIM_OK)
		return ERROR_FAIL;

	Jim_Obj *list = Jim_GetResult(interp);
	Jim_IncrRefCount(list);

	int len = Jim_ListLength(interp, list);
	for (int i = 0; i < len; i++) {
		Jim_Obj *elem = Jim_ListGetIndex(interp, list, i);
		Jim_IncrRefCount(elem);

		const char *name = Jim_GetString(elem, NULL);
		struct command *c = command_find_from_name(interp, name);
		if (!c) {
			/* not openocd command */
			Jim_DecrRefCount(interp, elem);
			continue;
		}
		if (false) /* too noisy with debug_level 3 */
			LOG_DEBUG("delete command \"%s\"", name);
#if JIM_VERSION >= 80
		Jim_DeleteCommand(interp, elem);
#else
		Jim_DeleteCommand(interp, name);
#endif

		help_del_command(cmd_ctx, name);

		Jim_DecrRefCount(interp, elem);
	}

	Jim_DecrRefCount(interp, list);
	return ERROR_OK;
}

int unregister_all_commands(struct command_context *context,
	const char *cmd_prefix)
{
	if (!context)
		return ERROR_OK;

	if (!cmd_prefix || !*cmd_prefix)
		return unregister_commands_match(context, "*");

	int retval = unregister_commands_match(context, "%s *", cmd_prefix);
	if (retval != ERROR_OK)
		return retval;

	return unregister_commands_match(context, "%s", cmd_prefix);
}

static int unregister_command(struct command_context *context,
	const char *cmd_prefix, const char *name)
{
	if (!context || !name)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!cmd_prefix || !*cmd_prefix)
		return unregister_commands_match(context, "%s", name);

	return unregister_commands_match(context, "%s %s", cmd_prefix, name);
}

void command_output_text(struct command_context *context, const char *data)
{
	if (context && context->output_handler && data)
		context->output_handler(context, data);
}

void command_print_sameline(struct command_invocation *cmd, const char *format, ...)
{
	char *string;

	va_list ap;
	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string != NULL && cmd) {
		/* we want this collected in the log + we also want to pick it up as a tcl return
		 * value.
		 *
		 * The latter bit isn't precisely neat, but will do for now.
		 */
		Jim_AppendString(cmd->ctx->interp, cmd->output, string, -1);
		/* We already printed it above
		 * command_output_text(context, string); */
		free(string);
	}

	va_end(ap);
}

void command_print(struct command_invocation *cmd, const char *format, ...)
{
	char *string;

	va_list ap;
	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string != NULL && cmd) {
		strcat(string, "\n");	/* alloc_vprintf guaranteed the buffer to be at least one
					 *char longer */
		/* we want this collected in the log + we also want to pick it up as a tcl return
		 * value.
		 *
		 * The latter bit isn't precisely neat, but will do for now.
		 */
		Jim_AppendString(cmd->ctx->interp, cmd->output, string, -1);
		/* We already printed it above
		 * command_output_text(context, string); */
		free(string);
	}

	va_end(ap);
}

static bool command_can_run(struct command_context *cmd_ctx, struct command *c, const char *full_name)
{
	if (c->mode == COMMAND_ANY || c->mode == cmd_ctx->mode)
		return true;

	/* Many commands may be run only before/after 'init' */
	const char *when;
	switch (c->mode) {
		case COMMAND_CONFIG:
			when = "before";
			break;
		case COMMAND_EXEC:
			when = "after";
			break;
		/* handle the impossible with humor; it guarantees a bug report! */
		default:
			when = "if Cthulhu is summoned by";
			break;
	}
	LOG_ERROR("The '%s' command must be used %s 'init'.",
			full_name ? full_name : c->name, when);
	return false;
}

static int run_command(struct command_context *context,
	struct command *c, const char **words, unsigned num_words)
{
	struct command_invocation cmd = {
		.ctx = context,
		.current = c,
		.name = c->name,
		.argc = num_words - 1,
		.argv = words + 1,
	};

	cmd.output = Jim_NewEmptyStringObj(context->interp);
	Jim_IncrRefCount(cmd.output);

	int retval = c->handler(&cmd);
	if (retval == ERROR_COMMAND_SYNTAX_ERROR) {
		/* Print help for command */
		command_run_linef(context, "usage %s", words[0]);
	} else if (retval == ERROR_COMMAND_CLOSE_CONNECTION) {
		/* just fall through for a shutdown request */
	} else {
		if (retval != ERROR_OK)
			LOG_DEBUG("Command '%s' failed with error code %d",
						words[0], retval);
		/* Use the command output as the Tcl result */
		Jim_SetResult(context->interp, cmd.output);
	}
	Jim_DecrRefCount(context->interp, cmd.output);

	return retval;
}

int command_run_line(struct command_context *context, char *line)
{
	/* all the parent commands have been registered with the interpreter
	 * so, can just evaluate the line as a script and check for
	 * results
	 */
	/* run the line thru a script engine */
	int retval = ERROR_FAIL;
	int retcode;
	/* Beware! This code needs to be reentrant. It is also possible
	 * for OpenOCD commands to be invoked directly from Tcl. This would
	 * happen when the Jim Tcl interpreter is provided by eCos for
	 * instance.
	 */
	struct target *saved_target_override = context->current_target_override;
	context->current_target_override = NULL;

	Jim_Interp *interp = context->interp;
	struct command_context *old_context = Jim_GetAssocData(interp, "context");
	Jim_DeleteAssocData(interp, "context");
	retcode = Jim_SetAssocData(interp, "context", NULL, context);
	if (retcode == JIM_OK) {
		/* associated the return value */
		Jim_DeleteAssocData(interp, "retval");
		retcode = Jim_SetAssocData(interp, "retval", NULL, &retval);
		if (retcode == JIM_OK) {
			retcode = Jim_Eval_Named(interp, line, 0, 0);

			Jim_DeleteAssocData(interp, "retval");
		}
		Jim_DeleteAssocData(interp, "context");
		int inner_retcode = Jim_SetAssocData(interp, "context", NULL, old_context);
		if (retcode == JIM_OK)
			retcode = inner_retcode;
	}
	context->current_target_override = saved_target_override;
	if (retcode == JIM_OK) {
		const char *result;
		int reslen;

		result = Jim_GetString(Jim_GetResult(interp), &reslen);
		if (reslen > 0) {
			command_output_text(context, result);
			command_output_text(context, "\n");
		}
		retval = ERROR_OK;
	} else if (retcode == JIM_EXIT) {
		/* ignore.
		 * exit(Jim_GetExitCode(interp)); */
	} else if (retcode == ERROR_COMMAND_CLOSE_CONNECTION) {
		return retcode;
	} else {
		Jim_MakeErrorMessage(interp);
		/* error is broadcast */
		LOG_USER("%s", Jim_GetString(Jim_GetResult(interp), NULL));

		if (retval == ERROR_OK) {
			/* It wasn't a low level OpenOCD command that failed */
			return ERROR_FAIL;
		}
		return retval;
	}

	return retval;
}

int command_run_linef(struct command_context *context, const char *format, ...)
{
	int retval = ERROR_FAIL;
	char *string;
	va_list ap;
	va_start(ap, format);
	string = alloc_vprintf(format, ap);
	if (string) {
		retval = command_run_line(context, string);
		free(string);
	}
	va_end(ap);
	return retval;
}

void command_set_output_handler(struct command_context *context,
	command_output_handler_t output_handler, void *priv)
{
	context->output_handler = output_handler;
	context->output_handler_priv = priv;
}

struct command_context *copy_command_context(struct command_context *context)
{
	struct command_context *copy_context = malloc(sizeof(struct command_context));

	*copy_context = *context;

	return copy_context;
}

void command_done(struct command_context *cmd_ctx)
{
	if (!cmd_ctx)
		return;

	free(cmd_ctx);
}

/* find full path to file */
static int jim_find(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 2)
		return JIM_ERR;
	const char *file = Jim_GetString(argv[1], NULL);
	char *full_path = find_file(file);
	if (!full_path)
		return JIM_ERR;
	Jim_Obj *result = Jim_NewStringObj(interp, full_path, strlen(full_path));
	free(full_path);

	Jim_SetResult(interp, result);
	return JIM_OK;
}

COMMAND_HANDLER(jim_echo)
{
	if (CMD_ARGC == 2 && !strcmp(CMD_ARGV[0], "-n")) {
		LOG_USER_N("%s", CMD_ARGV[1]);
		return JIM_OK;
	}
	if (CMD_ARGC != 1)
		return JIM_ERR;
	LOG_USER("%s", CMD_ARGV[0]);
	return JIM_OK;
}

/* Capture progress output and return as tcl return value. If the
 * progress output was empty, return tcl return value.
 */
static int jim_capture(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 2)
		return JIM_ERR;

	struct log_capture_state *state = command_log_capture_start(interp);

	/* disable polling during capture. This avoids capturing output
	 * from polling.
	 *
	 * This is necessary in order to avoid accidentally getting a non-empty
	 * string for tcl fn's.
	 */
	bool save_poll = jtag_poll_get_enabled();

	jtag_poll_set_enabled(false);

	const char *str = Jim_GetString(argv[1], NULL);
	int retcode = Jim_Eval_Named(interp, str, __THIS__FILE__, __LINE__);

	jtag_poll_set_enabled(save_poll);

	command_log_capture_finish(state);

	return retcode;
}

struct help_entry {
	struct list_head lh;
	char *cmd_name;
	char *help;
	char *usage;
};

static COMMAND_HELPER(command_help_show, struct help_entry *c,
	bool show_help, const char *cmd_match);

static COMMAND_HELPER(command_help_show_list, bool show_help, const char *cmd_match)
{
	struct help_entry *entry;

	list_for_each_entry(entry, CMD_CTX->help_list, lh)
		CALL_COMMAND_HANDLER(command_help_show, entry, show_help, cmd_match);
	return ERROR_OK;
}

#define HELP_LINE_WIDTH(_n) (int)(76 - (2 * _n))

static void command_help_show_indent(unsigned n)
{
	for (unsigned i = 0; i < n; i++)
		LOG_USER_N("  ");
}
static void command_help_show_wrap(const char *str, unsigned n, unsigned n2)
{
	const char *cp = str, *last = str;
	while (*cp) {
		const char *next = last;
		do {
			cp = next;
			do {
				next++;
			} while (*next != ' ' && *next != '\t' && *next != '\0');
		} while ((next - last < HELP_LINE_WIDTH(n)) && *next != '\0');
		if (next - last < HELP_LINE_WIDTH(n))
			cp = next;
		command_help_show_indent(n);
		LOG_USER("%.*s", (int)(cp - last), last);
		last = cp + 1;
		n = n2;
	}
}

static COMMAND_HELPER(command_help_show, struct help_entry *c,
	bool show_help, const char *cmd_match)
{
	unsigned int n = 0;
	for (const char *s = strchr(c->cmd_name, ' '); s; s = strchr(s + 1, ' '))
		n++;

	/* If the match string occurs anywhere, we print out
	 * stuff for this command. */
	bool is_match = (strstr(c->cmd_name, cmd_match) != NULL) ||
		((c->usage) && (strstr(c->usage, cmd_match) != NULL)) ||
		((c->help) && (strstr(c->help, cmd_match) != NULL));

	if (is_match) {
		if (c->usage && strlen(c->usage) > 0) {
			char *msg = alloc_printf("%s %s", c->cmd_name, c->usage);
			command_help_show_wrap(msg, n, n + 5);
			free(msg);
		} else {
			command_help_show_wrap(c->cmd_name, n, n + 5);
		}
	}

	if (is_match && show_help) {
		char *msg;

		/* TODO: factorize jim_command_mode() to avoid running jim command here */
		char *request = alloc_printf("command mode %s", c->cmd_name);
		if (!request) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		int retval = Jim_Eval(CMD_CTX->interp, request);
		free(request);
		enum command_mode mode = COMMAND_UNKNOWN;
		if (retval != JIM_ERR) {
			const char *result = Jim_GetString(Jim_GetResult(CMD_CTX->interp), NULL);
			if (!strcmp(result, "any"))
				mode = COMMAND_ANY;
			else if (!strcmp(result, "config"))
				mode = COMMAND_CONFIG;
			else if (!strcmp(result, "exec"))
				mode = COMMAND_EXEC;
		}

		/* Normal commands are runtime-only; highlight exceptions */
		if (mode != COMMAND_EXEC) {
			const char *stage_msg = "";

			switch (mode) {
				case COMMAND_CONFIG:
					stage_msg = " (configuration command)";
					break;
				case COMMAND_ANY:
					stage_msg = " (command valid any time)";
					break;
				default:
					stage_msg = " (?mode error?)";
					break;
			}
			msg = alloc_printf("%s%s", c->help ? c->help : "", stage_msg);
		} else
			msg = alloc_printf("%s", c->help ? c->help : "");

		if (msg) {
			command_help_show_wrap(msg, n + 3, n + 3);
			free(msg);
		} else
			return -ENOMEM;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_help_command)
{
	bool full = strcmp(CMD_NAME, "help") == 0;
	int retval;
	char *cmd_match;

	if (CMD_ARGC <= 0)
		cmd_match = strdup("");

	else {
		cmd_match = strdup(CMD_ARGV[0]);

		for (unsigned int i = 1; i < CMD_ARGC && cmd_match; ++i) {
			char *prev = cmd_match;
			cmd_match = alloc_printf("%s %s", prev, CMD_ARGV[i]);
			free(prev);
		}
	}

	if (!cmd_match) {
		LOG_ERROR("unable to build search string");
		return -ENOMEM;
	}
	retval = CALL_COMMAND_HANDLER(command_help_show_list, full, cmd_match);

	free(cmd_match);
	return retval;
}

static char *alloc_concatenate_strings(int argc, Jim_Obj * const *argv)
{
	char *prev, *all;
	int i;

	assert(argc >= 1);

	all = strdup(Jim_GetString(argv[0], NULL));
	if (!all) {
		LOG_ERROR("Out of memory");
		return NULL;
	}

	for (i = 1; i < argc; ++i) {
		prev = all;
		all = alloc_printf("%s %s", all, Jim_GetString(argv[i], NULL));
		free(prev);
		if (!all) {
			LOG_ERROR("Out of memory");
			return NULL;
		}
	}

	return all;
}

static int exec_command(Jim_Interp *interp, struct command_context *cmd_ctx,
		struct command *c, int argc, Jim_Obj * const *argv)
{
	if (c->jim_handler)
		return c->jim_handler(interp, argc, argv);

	/* use c->handler */
	unsigned int nwords;
	char **words = script_command_args_alloc(argc, argv, &nwords);
	if (!words)
		return JIM_ERR;

	int retval = run_command(cmd_ctx, c, (const char **)words, nwords);
	script_command_args_free(words, nwords);
	return command_retval_set(interp, retval);
}

static int jim_command_dispatch(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	script_debug(interp, argc, argv);

	/* check subcommands */
	if (argc > 1) {
		char *s = alloc_printf("%s %s", Jim_GetString(argv[0], NULL), Jim_GetString(argv[1], NULL));
		Jim_Obj *js = Jim_NewStringObj(interp, s, -1);
		Jim_IncrRefCount(js);
		free(s);
		Jim_Cmd *cmd = Jim_GetCommand(interp, js, JIM_NONE);
		if (cmd) {
			int retval = Jim_EvalObjPrefix(interp, js, argc - 2, argv + 2);
			Jim_DecrRefCount(interp, js);
			return retval;
		}
		Jim_DecrRefCount(interp, js);
	}

	struct command *c = jim_to_command(interp);
	if (!c->jim_handler && !c->handler) {
		Jim_EvalObjPrefix(interp, Jim_NewStringObj(interp, "usage", -1), 1, argv);
		return JIM_ERR;
	}

	struct command_context *cmd_ctx = current_command_context(interp);

	if (!command_can_run(cmd_ctx, c, Jim_GetString(argv[0], NULL)))
		return JIM_ERR;

	target_call_timer_callbacks_now();

	/*
	 * Black magic of overridden current target:
	 * If the command we are going to handle has a target prefix,
	 * override the current target temporarily for the time
	 * of processing the command.
	 * current_target_override is used also for event handlers
	 * therefore we prevent touching it if command has no prefix.
	 * Previous override is saved and restored back to ensure
	 * correct work when jim_command_dispatch() is re-entered.
	 */
	struct target *saved_target_override = cmd_ctx->current_target_override;
	if (c->jim_override_target)
		cmd_ctx->current_target_override = c->jim_override_target;

	int retval = exec_command(interp, cmd_ctx, c, argc, argv);

	if (c->jim_override_target)
		cmd_ctx->current_target_override = saved_target_override;

	return retval;
}

static int jim_command_mode(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct command_context *cmd_ctx = current_command_context(interp);
	enum command_mode mode;

	if (argc > 1) {
		char *full_name = alloc_concatenate_strings(argc - 1, argv + 1);
		if (!full_name)
			return JIM_ERR;
		Jim_Obj *s = Jim_NewStringObj(interp, full_name, -1);
		Jim_IncrRefCount(s);
		Jim_Cmd *cmd = Jim_GetCommand(interp, s, JIM_NONE);
		Jim_DecrRefCount(interp, s);
		free(full_name);
		if (!cmd || !(jimcmd_is_proc(cmd) || jimcmd_is_oocd_command(cmd))) {
			Jim_SetResultString(interp, "unknown", -1);
			return JIM_OK;
		}

		if (jimcmd_is_proc(cmd)) {
			/* tcl proc */
			mode = COMMAND_ANY;
		} else {
			struct command *c = jimcmd_privdata(cmd);

			mode = c->mode;
		}
	} else
		mode = cmd_ctx->mode;

	const char *mode_str;
	switch (mode) {
		case COMMAND_ANY:
			mode_str = "any";
			break;
		case COMMAND_CONFIG:
			mode_str = "config";
			break;
		case COMMAND_EXEC:
			mode_str = "exec";
			break;
		default:
			mode_str = "unknown";
			break;
	}
	Jim_SetResultString(interp, mode_str, -1);
	return JIM_OK;
}

int help_del_all_commands(struct command_context *cmd_ctx)
{
	struct help_entry *curr, *n;

	list_for_each_entry_safe(curr, n, cmd_ctx->help_list, lh) {
		list_del(&curr->lh);
		free(curr->cmd_name);
		free(curr->help);
		free(curr->usage);
		free(curr);
	}
	return ERROR_OK;
}

static int help_del_command(struct command_context *cmd_ctx, const char *cmd_name)
{
	struct help_entry *curr;

	list_for_each_entry(curr, cmd_ctx->help_list, lh) {
		if (!strcmp(cmd_name, curr->cmd_name)) {
			list_del(&curr->lh);
			free(curr->cmd_name);
			free(curr->help);
			free(curr->usage);
			free(curr);
			break;
		}
	}

	return ERROR_OK;
}

static int help_add_command(struct command_context *cmd_ctx,
	const char *cmd_name, const char *help_text, const char *usage_text)
{
	int cmp = -1; /* add after curr */
	struct help_entry *curr;

	list_for_each_entry_reverse(curr, cmd_ctx->help_list, lh) {
		cmp = strcmp(cmd_name, curr->cmd_name);
		if (cmp >= 0)
			break;
	}

	struct help_entry *entry;
	if (cmp) {
		entry = calloc(1, sizeof(*entry));
		if (!entry) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		entry->cmd_name = strdup(cmd_name);
		if (!entry->cmd_name) {
			LOG_ERROR("Out of memory");
			free(entry);
			return ERROR_FAIL;
		}
		list_add(&entry->lh, &curr->lh);
	} else {
		entry = curr;
	}

	if (help_text) {
		char *text = strdup(help_text);
		if (!text) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		free(entry->help);
		entry->help = text;
	}

	if (usage_text) {
		char *text = strdup(usage_text);
		if (!text) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		free(entry->usage);
		entry->usage = text;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_help_add_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	const char *help = !strcmp(CMD_NAME, "add_help_text") ? CMD_ARGV[1] : NULL;
	const char *usage = !strcmp(CMD_NAME, "add_usage_text") ? CMD_ARGV[1] : NULL;
	if (!help && !usage) {
		LOG_ERROR("command name '%s' is unknown", CMD_NAME);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	const char *cmd_name = CMD_ARGV[0];
	return help_add_command(CMD_CTX, cmd_name, help, usage);
}

/* sleep command sleeps for <n> milliseconds
 * this is useful in target startup scripts
 */
COMMAND_HANDLER(handle_sleep_command)
{
	bool busy = false;
	if (CMD_ARGC == 2) {
		if (strcmp(CMD_ARGV[1], "busy") == 0)
			busy = true;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	} else if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned long duration = 0;
	int retval = parse_ulong(CMD_ARGV[0], &duration);
	if (retval != ERROR_OK)
		return retval;

	if (!busy) {
		int64_t then = timeval_ms();
		while (timeval_ms() - then < (int64_t)duration) {
			target_call_timer_callbacks_now();
			usleep(1000);
		}
	} else
		busy_sleep(duration);

	return ERROR_OK;
}

static const struct command_registration command_subcommand_handlers[] = {
	{
		.name = "mode",
		.mode = COMMAND_ANY,
		.jim_handler = jim_command_mode,
		.usage = "[command_name ...]",
		.help = "Returns the command modes allowed by a command: "
			"'any', 'config', or 'exec'. If no command is "
			"specified, returns the current command mode. "
			"Returns 'unknown' if an unknown command is given. "
			"Command can be multiple tokens.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration command_builtin_handlers[] = {
	{
		.name = "ocd_find",
		.mode = COMMAND_ANY,
		.jim_handler = jim_find,
		.help = "find full path to file",
		.usage = "file",
	},
	{
		.name = "capture",
		.mode = COMMAND_ANY,
		.jim_handler = jim_capture,
		.help = "Capture progress output and return as tcl return value. If the "
				"progress output was empty, return tcl return value.",
		.usage = "command",
	},
	{
		.name = "echo",
		.handler = jim_echo,
		.mode = COMMAND_ANY,
		.help = "Logs a message at \"user\" priority. "
			"Option \"-n\" suppresses trailing newline",
		.usage = "[-n] string",
	},
	{
		.name = "add_help_text",
		.handler = handle_help_add_command,
		.mode = COMMAND_ANY,
		.help = "Add new command help text; "
			"Command can be multiple tokens.",
		.usage = "command_name helptext_string",
	},
	{
		.name = "add_usage_text",
		.handler = handle_help_add_command,
		.mode = COMMAND_ANY,
		.help = "Add new command usage text; "
			"command can be multiple tokens.",
		.usage = "command_name usage_string",
	},
	{
		.name = "sleep",
		.handler = handle_sleep_command,
		.mode = COMMAND_ANY,
		.help = "Sleep for specified number of milliseconds.  "
			"\"busy\" will busy wait instead (avoid this).",
		.usage = "milliseconds ['busy']",
	},
	{
		.name = "help",
		.handler = handle_help_command,
		.mode = COMMAND_ANY,
		.help = "Show full command help; "
			"command can be multiple tokens.",
		.usage = "[command_name]",
	},
	{
		.name = "usage",
		.handler = handle_help_command,
		.mode = COMMAND_ANY,
		.help = "Show basic command usage; "
			"command can be multiple tokens.",
		.usage = "[command_name]",
	},
	{
		.name = "command",
		.mode = COMMAND_ANY,
		.help = "core command group (introspection)",
		.chain = command_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

struct command_context *command_init(const char *startup_tcl, Jim_Interp *interp)
{
	struct command_context *context = calloc(1, sizeof(struct command_context));

	context->mode = COMMAND_EXEC;

	/* context can be duplicated. Put list head on separate mem-chunk to keep list consistent */
	context->help_list = malloc(sizeof(*context->help_list));
	INIT_LIST_HEAD(context->help_list);

	/* Create a jim interpreter if we were not handed one */
	if (!interp) {
		/* Create an interpreter */
		interp = Jim_CreateInterp();
		/* Add all the Jim core commands */
		Jim_RegisterCoreCommands(interp);
		Jim_InitStaticExtensions(interp);
	}

	context->interp = interp;

	register_commands(context, NULL, command_builtin_handlers);

	Jim_SetAssocData(interp, "context", NULL, context);
	if (Jim_Eval_Named(interp, startup_tcl, "embedded:startup.tcl", 1) == JIM_ERR) {
		LOG_ERROR("Failed to run startup.tcl (embedded into OpenOCD)");
		Jim_MakeErrorMessage(interp);
		LOG_USER_N("%s", Jim_GetString(Jim_GetResult(interp), NULL));
		exit(-1);
	}
	Jim_DeleteAssocData(interp, "context");

	return context;
}

void command_exit(struct command_context *context)
{
	if (!context)
		return;

	Jim_FreeInterp(context->interp);
	free(context->help_list);
	command_done(context);
}

int command_context_mode(struct command_context *cmd_ctx, enum command_mode mode)
{
	if (!cmd_ctx)
		return ERROR_COMMAND_SYNTAX_ERROR;

	cmd_ctx->mode = mode;
	return ERROR_OK;
}

void process_jim_events(struct command_context *cmd_ctx)
{
	static int recursion;
	if (recursion)
		return;

	recursion++;
	Jim_ProcessEvents(cmd_ctx->interp, JIM_ALL_EVENTS | JIM_DONT_WAIT);
	recursion--;
}

#define DEFINE_PARSE_NUM_TYPE(name, type, func, min, max) \
	int parse ## name(const char *str, type * ul) \
	{ \
		if (!*str) { \
			LOG_ERROR("Invalid command argument"); \
			return ERROR_COMMAND_ARGUMENT_INVALID; \
		} \
		char *end; \
		errno = 0; \
		*ul = func(str, &end, 0); \
		if (*end) { \
			LOG_ERROR("Invalid command argument"); \
			return ERROR_COMMAND_ARGUMENT_INVALID; \
		} \
		if ((max == *ul) && (errno == ERANGE)) { \
			LOG_ERROR("Argument overflow");	\
			return ERROR_COMMAND_ARGUMENT_OVERFLOW;	\
		} \
		if (min && (min == *ul) && (errno == ERANGE)) { \
			LOG_ERROR("Argument underflow"); \
			return ERROR_COMMAND_ARGUMENT_UNDERFLOW; \
		} \
		return ERROR_OK; \
	}
DEFINE_PARSE_NUM_TYPE(_ulong, unsigned long, strtoul, 0, ULONG_MAX)
DEFINE_PARSE_NUM_TYPE(_ullong, unsigned long long, strtoull, 0, ULLONG_MAX)
DEFINE_PARSE_NUM_TYPE(_long, long, strtol, LONG_MIN, LONG_MAX)
DEFINE_PARSE_NUM_TYPE(_llong, long long, strtoll, LLONG_MIN, LLONG_MAX)

#define DEFINE_PARSE_WRAPPER(name, type, min, max, functype, funcname) \
	int parse ## name(const char *str, type * ul) \
	{ \
		functype n; \
		int retval = parse ## funcname(str, &n); \
		if (retval != ERROR_OK)	\
			return retval; \
		if (n > max) \
			return ERROR_COMMAND_ARGUMENT_OVERFLOW;	\
		if (min) \
			return ERROR_COMMAND_ARGUMENT_UNDERFLOW; \
		*ul = n; \
		return ERROR_OK; \
	}

#define DEFINE_PARSE_ULONGLONG(name, type, min, max) \
	DEFINE_PARSE_WRAPPER(name, type, min, max, unsigned long long, _ullong)
DEFINE_PARSE_ULONGLONG(_uint, unsigned, 0, UINT_MAX)
DEFINE_PARSE_ULONGLONG(_u64,  uint64_t, 0, UINT64_MAX)
DEFINE_PARSE_ULONGLONG(_u32,  uint32_t, 0, UINT32_MAX)
DEFINE_PARSE_ULONGLONG(_u16,  uint16_t, 0, UINT16_MAX)
DEFINE_PARSE_ULONGLONG(_u8,   uint8_t,  0, UINT8_MAX)

DEFINE_PARSE_ULONGLONG(_target_addr, target_addr_t, 0, TARGET_ADDR_MAX)

#define DEFINE_PARSE_LONGLONG(name, type, min, max) \
	DEFINE_PARSE_WRAPPER(name, type, min, max, long long, _llong)
DEFINE_PARSE_LONGLONG(_int, int,     n < INT_MIN,   INT_MAX)
DEFINE_PARSE_LONGLONG(_s64, int64_t, n < INT64_MIN, INT64_MAX)
DEFINE_PARSE_LONGLONG(_s32, int32_t, n < INT32_MIN, INT32_MAX)
DEFINE_PARSE_LONGLONG(_s16, int16_t, n < INT16_MIN, INT16_MAX)
DEFINE_PARSE_LONGLONG(_s8,  int8_t,  n < INT8_MIN,  INT8_MAX)

static int command_parse_bool(const char *in, bool *out,
	const char *on, const char *off)
{
	if (strcasecmp(in, on) == 0)
		*out = true;
	else if (strcasecmp(in, off) == 0)
		*out = false;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;
	return ERROR_OK;
}

int command_parse_bool_arg(const char *in, bool *out)
{
	if (command_parse_bool(in, out, "on", "off") == ERROR_OK)
		return ERROR_OK;
	if (command_parse_bool(in, out, "enable", "disable") == ERROR_OK)
		return ERROR_OK;
	if (command_parse_bool(in, out, "true", "false") == ERROR_OK)
		return ERROR_OK;
	if (command_parse_bool(in, out, "yes", "no") == ERROR_OK)
		return ERROR_OK;
	if (command_parse_bool(in, out, "1", "0") == ERROR_OK)
		return ERROR_OK;
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HELPER(handle_command_parse_bool, bool *out, const char *label)
{
	switch (CMD_ARGC) {
		case 1: {
			const char *in = CMD_ARGV[0];
			if (command_parse_bool_arg(in, out) != ERROR_OK) {
				LOG_ERROR("%s: argument '%s' is not valid", CMD_NAME, in);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		}
			/* fallthrough */
		case 0:
			LOG_INFO("%s is %s", label, *out ? "enabled" : "disabled");
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}
