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

static int run_command(struct command_context *context,
		struct command *c, const char *words[], unsigned num_words);

struct log_capture_state {
	Jim_Interp *interp;
	Jim_Obj *output;
};

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
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);
	if (NULL == tclOutput)
		return NULL;

	struct log_capture_state *state = malloc(sizeof(*state));
	if (NULL == state)
		return NULL;

	state->interp = interp;
	Jim_IncrRefCount(tclOutput);
	state->output = tclOutput;

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
	if (NULL == state)
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

static int command_retval_set(Jim_Interp *interp, int retval)
{
	int *return_retval = Jim_GetAssocData(interp, "retval");
	if (return_retval != NULL)
		*return_retval = retval;

	return (retval == ERROR_OK) ? JIM_OK : retval;
}

extern struct command_context *global_cmd_ctx;

/* dump a single line to the log for the command.
 * Do nothing in case we are not at debug level 3 */
void script_debug(Jim_Interp *interp, const char *name,
	unsigned argc, Jim_Obj * const *argv)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	char *dbg = alloc_printf("command - %s", name);
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
	if (NULL == words)
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
	if (NULL == cmd_ctx) {
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

static int script_command_run(Jim_Interp *interp,
	int argc, Jim_Obj * const *argv, struct command *c)
{
	target_call_timer_callbacks_now();
	LOG_USER_N("%s", "");	/* Keep GDB connection alive*/

	unsigned nwords;
	char **words = script_command_args_alloc(argc, argv, &nwords);
	if (NULL == words)
		return JIM_ERR;

	struct command_context *cmd_ctx = current_command_context(interp);
	int retval = run_command(cmd_ctx, c, (const char **)words, nwords);

	script_command_args_free(words, nwords);
	return command_retval_set(interp, retval);
}

static int script_command(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	/* the private data is stashed in the interp structure */

	struct command *c = interp->cmdPrivData;
	assert(c);
	script_debug(interp, c->name, argc, argv);
	return script_command_run(interp, argc, argv, c);
}

static struct command *command_root(struct command *c)
{
	while (NULL != c->parent)
		c = c->parent;
	return c;
}

/**
 * Find a command by name from a list of commands.
 * @returns Returns the named command if it exists in the list.
 * Returns NULL otherwise.
 */
static struct command *command_find(struct command *head, const char *name)
{
	for (struct command *cc = head; cc; cc = cc->next) {
		if (strcmp(cc->name, name) == 0)
			return cc;
	}
	return NULL;
}

struct command *command_find_in_context(struct command_context *cmd_ctx,
	const char *name)
{
	return command_find(cmd_ctx->commands, name);
}
struct command *command_find_in_parent(struct command *parent,
	const char *name)
{
	return command_find(parent->children, name);
}

/**
 * Add the command into the linked list, sorted by name.
 * @param head Address to head of command list pointer, which may be
 * updated if @c c gets inserted at the beginning of the list.
 * @param c The command to add to the list pointed to by @c head.
 */
static void command_add_child(struct command **head, struct command *c)
{
	assert(head);
	if (NULL == *head) {
		*head = c;
		return;
	}

	while ((*head)->next && (strcmp(c->name, (*head)->name) > 0))
		head = &(*head)->next;

	if (strcmp(c->name, (*head)->name) > 0) {
		c->next = (*head)->next;
		(*head)->next = c;
	} else {
		c->next = *head;
		*head = c;
	}
}

static struct command **command_list_for_parent(
	struct command_context *cmd_ctx, struct command *parent)
{
	return parent ? &parent->children : &cmd_ctx->commands;
}

static void command_free(struct command *c)
{
	/** @todo if command has a handler, unregister its jim command! */

	while (NULL != c->children) {
		struct command *tmp = c->children;
		c->children = tmp->next;
		command_free(tmp);
	}

	free(c->name);
	free(c->help);
	free(c->usage);
	free(c);
}

static struct command *command_new(struct command_context *cmd_ctx,
	struct command *parent, const struct command_registration *cr)
{
	assert(cr->name);

	/*
	 * If it is a non-jim command with no .usage specified,
	 * log an error.
	 *
	 * strlen(.usage) == 0 means that the command takes no
	 * arguments.
	*/
	if ((cr->jim_handler == NULL) && (cr->usage == NULL)) {
		LOG_ERROR("BUG: command '%s%s%s' does not have the "
			"'.usage' field filled out",
			parent && parent->name ? parent->name : "",
			parent && parent->name ? " " : "",
			cr->name);
	}

	struct command *c = calloc(1, sizeof(struct command));
	if (NULL == c)
		return NULL;

	c->name = strdup(cr->name);
	if (cr->help)
		c->help = strdup(cr->help);
	if (cr->usage)
		c->usage = strdup(cr->usage);

	if (!c->name || (cr->help && !c->help) || (cr->usage && !c->usage))
		goto command_new_error;

	c->parent = parent;
	c->handler = cr->handler;
	c->jim_handler = cr->jim_handler;
	c->jim_handler_data = cr->jim_handler_data;
	c->mode = cr->mode;

	command_add_child(command_list_for_parent(cmd_ctx, parent), c);

	return c;

command_new_error:
	command_free(c);
	return NULL;
}

static int command_unknown(Jim_Interp *interp, int argc, Jim_Obj *const *argv);

static int register_command_handler(struct command_context *cmd_ctx,
	struct command *c)
{
	Jim_Interp *interp = cmd_ctx->interp;

	LOG_DEBUG("registering '%s'...", c->name);

	Jim_CmdProc *func = c->handler ? &script_command : &command_unknown;
	int retval = Jim_CreateCommand(interp, c->name, func, c, NULL);

	return retval;
}

struct command *register_command(struct command_context *context,
	struct command *parent, const struct command_registration *cr)
{
	if (!context || !cr->name)
		return NULL;

	const char *name = cr->name;
	struct command **head = command_list_for_parent(context, parent);
	struct command *c = command_find(*head, name);
	if (NULL != c) {
		/* TODO: originally we treated attempting to register a cmd twice as an error
		 * Sometimes we need this behaviour, such as with flash banks.
		 * http://www.mail-archive.com/openocd-development@lists.berlios.de/msg11152.html */
		LOG_DEBUG("command '%s' is already registered in '%s' context",
			name, parent ? parent->name : "<global>");
		return c;
	}

	c = command_new(context, parent, cr);
	if (NULL == c)
		return NULL;

	int retval = ERROR_OK;
	if (NULL != cr->jim_handler && NULL == parent) {
		retval = Jim_CreateCommand(context->interp, cr->name,
				cr->jim_handler, cr->jim_handler_data, NULL);
	} else if (NULL != cr->handler || NULL != parent)
		retval = register_command_handler(context, command_root(c));

	if (ERROR_OK != retval) {
		unregister_command(context, parent, name);
		c = NULL;
	}
	return c;
}

int register_commands(struct command_context *cmd_ctx, struct command *parent,
	const struct command_registration *cmds)
{
	int retval = ERROR_OK;
	unsigned i;
	for (i = 0; cmds[i].name || cmds[i].chain; i++) {
		const struct command_registration *cr = cmds + i;

		struct command *c = NULL;
		if (NULL != cr->name) {
			c = register_command(cmd_ctx, parent, cr);
			if (NULL == c) {
				retval = ERROR_FAIL;
				break;
			}
		}
		if (NULL != cr->chain) {
			struct command *p = c ? : parent;
			retval = register_commands(cmd_ctx, p, cr->chain);
			if (ERROR_OK != retval)
				break;
		}
	}
	if (ERROR_OK != retval) {
		for (unsigned j = 0; j < i; j++)
			unregister_command(cmd_ctx, parent, cmds[j].name);
	}
	return retval;
}

int unregister_all_commands(struct command_context *context,
	struct command *parent)
{
	if (context == NULL)
		return ERROR_OK;

	struct command **head = command_list_for_parent(context, parent);
	while (NULL != *head) {
		struct command *tmp = *head;
		*head = tmp->next;
		command_free(tmp);
	}

	return ERROR_OK;
}

int unregister_command(struct command_context *context,
	struct command *parent, const char *name)
{
	if ((!context) || (!name))
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct command *p = NULL;
	struct command **head = command_list_for_parent(context, parent);
	for (struct command *c = *head; NULL != c; p = c, c = c->next) {
		if (strcmp(name, c->name) != 0)
			continue;

		if (p)
			p->next = c->next;
		else
			*head = c->next;

		command_free(c);
		return ERROR_OK;
	}

	return ERROR_OK;
}

void command_set_handler_data(struct command *c, void *p)
{
	if (NULL != c->handler || NULL != c->jim_handler)
		c->jim_handler_data = p;
	for (struct command *cc = c->children; NULL != cc; cc = cc->next)
		command_set_handler_data(cc, p);
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

static char *__command_name(struct command *c, char delim, unsigned extra)
{
	char *name;
	unsigned len = strlen(c->name);
	if (NULL == c->parent) {
		/* allocate enough for the name, child names, and '\0' */
		name = malloc(len + extra + 1);
		if (!name) {
			LOG_ERROR("Out of memory");
			return NULL;
		}
		strcpy(name, c->name);
	} else {
		/* parent's extra must include both the space and name */
		name = __command_name(c->parent, delim, 1 + len + extra);
		char dstr[2] = { delim, 0 };
		strcat(name, dstr);
		strcat(name, c->name);
	}
	return name;
}

char *command_name(struct command *c, char delim)
{
	return __command_name(c, delim, 0);
}

static bool command_can_run(struct command_context *cmd_ctx, struct command *c)
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
	char *full_name = command_name(c, ' ');
	LOG_ERROR("The '%s' command must be used %s 'init'.",
			full_name ? full_name : c->name, when);
	free(full_name);
	return false;
}

static int run_command(struct command_context *context,
	struct command *c, const char *words[], unsigned num_words)
{
	if (!command_can_run(context, c))
		return ERROR_FAIL;

	struct command_invocation cmd = {
		.ctx = context,
		.current = c,
		.name = c->name,
		.argc = num_words - 1,
		.argv = words + 1,
	};
	/* Black magic of overridden current target:
	 * If the command we are going to handle has a target prefix,
	 * override the current target temporarily for the time
	 * of processing the command.
	 * current_target_override is used also for event handlers
	 * therefore we prevent touching it if command has no prefix.
	 * Previous override is saved and restored back to ensure
	 * correct work when run_command() is re-entered. */
	struct target *saved_target_override = context->current_target_override;
	if (c->jim_handler_data)
		context->current_target_override = c->jim_handler_data;

	cmd.output = Jim_NewEmptyStringObj(context->interp);
	Jim_IncrRefCount(cmd.output);

	int retval = c->handler(&cmd);

	if (c->jim_handler_data)
		context->current_target_override = saved_target_override;

	if (retval == ERROR_COMMAND_SYNTAX_ERROR) {
		/* Print help for command */
		char *full_name = command_name(c, ' ');
		if (NULL != full_name) {
			command_run_linef(context, "usage %s", full_name);
			free(full_name);
		}
	} else if (retval == ERROR_COMMAND_CLOSE_CONNECTION) {
		/* just fall through for a shutdown request */
	} else {
		if (retval != ERROR_OK) {
			char *full_name = command_name(c, ' ');
			LOG_DEBUG("Command '%s' failed with error code %d",
						full_name ? full_name : c->name, retval);
			free(full_name);
		}
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
	if (string != NULL) {
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
	if (NULL == cmd_ctx)
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
	if (full_path == NULL)
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

static COMMAND_HELPER(command_help_find, struct command *head,
	struct command **out)
{
	if (0 == CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;
	*out = command_find(head, CMD_ARGV[0]);
	if (NULL == *out)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (--CMD_ARGC == 0)
		return ERROR_OK;
	CMD_ARGV++;
	return CALL_COMMAND_HANDLER(command_help_find, (*out)->children, out);
}

static COMMAND_HELPER(command_help_show, struct command *c, unsigned n,
	bool show_help, const char *cmd_match);

static COMMAND_HELPER(command_help_show_list, struct command *head, unsigned n,
	bool show_help, const char *cmd_match)
{
	for (struct command *c = head; NULL != c; c = c->next)
		CALL_COMMAND_HANDLER(command_help_show, c, n, show_help, cmd_match);
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

static COMMAND_HELPER(command_help_show, struct command *c, unsigned n,
	bool show_help, const char *cmd_match)
{
	char *cmd_name = command_name(c, ' ');
	if (NULL == cmd_name)
		return ERROR_FAIL;

	/* If the match string occurs anywhere, we print out
	 * stuff for this command. */
	bool is_match = (strstr(cmd_name, cmd_match) != NULL) ||
		((c->usage != NULL) && (strstr(c->usage, cmd_match) != NULL)) ||
		((c->help != NULL) && (strstr(c->help, cmd_match) != NULL));

	if (is_match) {
		command_help_show_indent(n);
		LOG_USER_N("%s", cmd_name);
	}
	free(cmd_name);

	if (is_match) {
		if (c->usage && strlen(c->usage) > 0) {
			LOG_USER_N(" ");
			command_help_show_wrap(c->usage, 0, n + 5);
		} else
			LOG_USER_N("\n");
	}

	if (is_match && show_help) {
		char *msg;

		/* Normal commands are runtime-only; highlight exceptions */
		if (c->mode != COMMAND_EXEC) {
			const char *stage_msg = "";

			switch (c->mode) {
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
			msg = alloc_printf("%s%s", c->help ? : "", stage_msg);
		} else
			msg = alloc_printf("%s", c->help ? : "");

		if (NULL != msg) {
			command_help_show_wrap(msg, n + 3, n + 3);
			free(msg);
		} else
			return -ENOMEM;
	}

	if (++n > 5) {
		LOG_ERROR("command recursion exceeded");
		return ERROR_FAIL;
	}

	return CALL_COMMAND_HANDLER(command_help_show_list,
		c->children, n, show_help, cmd_match);
}

COMMAND_HANDLER(handle_help_command)
{
	bool full = strcmp(CMD_NAME, "help") == 0;
	int retval;
	struct command *c = CMD_CTX->commands;
	char *cmd_match = NULL;

	if (CMD_ARGC == 0)
		cmd_match = "";
	else if (CMD_ARGC >= 1) {
		unsigned i;

		for (i = 0; i < CMD_ARGC; ++i) {
			if (NULL != cmd_match) {
				char *prev = cmd_match;

				cmd_match = alloc_printf("%s %s", cmd_match, CMD_ARGV[i]);
				free(prev);
				if (NULL == cmd_match) {
					LOG_ERROR("unable to build search string");
					return -ENOMEM;
				}
			} else {
				cmd_match = alloc_printf("%s", CMD_ARGV[i]);
				if (NULL == cmd_match) {
					LOG_ERROR("unable to build search string");
					return -ENOMEM;
				}
			}
		}
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	retval = CALL_COMMAND_HANDLER(command_help_show_list,
			c, 0, full, cmd_match);

	if (CMD_ARGC >= 1)
		free(cmd_match);
	return retval;
}

static int command_unknown_find(unsigned argc, Jim_Obj *const *argv,
	struct command *head, struct command **out)
{
	if (0 == argc)
		return argc;
	const char *cmd_name = Jim_GetString(argv[0], NULL);
	struct command *c = command_find(head, cmd_name);
	if (NULL == c)
		return argc;
	*out = c;
	return command_unknown_find(--argc, ++argv, (*out)->children, out);
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

static int run_usage(Jim_Interp *interp, int argc_valid, int argc, Jim_Obj * const *argv)
{
	struct command_context *cmd_ctx = current_command_context(interp);
	char *command;
	int retval;

	assert(argc_valid >= 1);
	assert(argc >= argc_valid);

	command = alloc_concatenate_strings(argc_valid, argv);
	if (!command)
		return JIM_ERR;

	retval = command_run_linef(cmd_ctx, "usage %s", command);
	if (retval != ERROR_OK) {
		LOG_ERROR("unable to execute command \"usage %s\"", command);
		return JIM_ERR;
	}

	if (argc_valid == argc)
		LOG_ERROR("%s: command requires more arguments", command);
	else {
		free(command);
		command = alloc_concatenate_strings(argc - argc_valid, argv + argc_valid);
		if (!command)
			return JIM_ERR;
		LOG_ERROR("invalid subcommand \"%s\"", command);
	}

	free(command);
	return retval;
}

static int command_unknown(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);
	script_debug(interp, cmd_name, argc, argv);

	struct command_context *cmd_ctx = current_command_context(interp);
	struct command *c = cmd_ctx->commands;
	int remaining = command_unknown_find(argc, argv, c, &c);
	/* if nothing could be consumed, then it's really an unknown command */
	if (remaining == argc) {
		const char *cmd = Jim_GetString(argv[0], NULL);
		LOG_ERROR("Unknown command:\n  %s", cmd);
		return JIM_OK;
	}

	Jim_Obj *const *start;
	unsigned count;
	if (c->handler || c->jim_handler) {
		/* include the command name in the list */
		count = remaining + 1;
		start = argv + (argc - remaining - 1);
	} else {
		count = argc - remaining;
		start = argv;
		run_usage(interp, count, argc, start);
		return JIM_ERR;
	}
	/* pass the command through to the intended handler */
	if (c->jim_handler) {
		if (!command_can_run(cmd_ctx, c))
			return JIM_ERR;

		interp->cmdPrivData = c->jim_handler_data;
		return (*c->jim_handler)(interp, count, start);
	}

	return script_command_run(interp, count, start, c);
}

static int jim_command_mode(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct command_context *cmd_ctx = current_command_context(interp);
	enum command_mode mode;

	if (argc > 1) {
		struct command *c = cmd_ctx->commands;
		int remaining = command_unknown_find(argc - 1, argv + 1, c, &c);
		/* if nothing could be consumed, then it's an unknown command */
		if (remaining == argc - 1) {
			Jim_SetResultString(interp, "unknown", -1);
			return JIM_OK;
		}
		mode = c->mode;
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

int help_add_command(struct command_context *cmd_ctx, struct command *parent,
	const char *cmd_name, const char *help_text, const char *usage)
{
	struct command **head = command_list_for_parent(cmd_ctx, parent);
	struct command *nc = command_find(*head, cmd_name);
	if (NULL == nc) {
		/* add a new command with help text */
		struct command_registration cr = {
			.name = cmd_name,
			.mode = COMMAND_ANY,
			.help = help_text,
			.usage = usage ? : "",
		};
		nc = register_command(cmd_ctx, parent, &cr);
		if (NULL == nc) {
			LOG_ERROR("failed to add '%s' help text", cmd_name);
			return ERROR_FAIL;
		}
		LOG_DEBUG("added '%s' help text", cmd_name);
		return ERROR_OK;
	}
	if (help_text) {
		bool replaced = false;
		if (nc->help) {
			free(nc->help);
			replaced = true;
		}
		nc->help = strdup(help_text);
		if (replaced)
			LOG_INFO("replaced existing '%s' help", cmd_name);
		else
			LOG_DEBUG("added '%s' help text", cmd_name);
	}
	if (usage) {
		bool replaced = false;
		if (nc->usage) {
			if (*nc->usage)
				replaced = true;
			free(nc->usage);
		}
		nc->usage = strdup(usage);
		if (replaced)
			LOG_INFO("replaced existing '%s' usage", cmd_name);
		else
			LOG_DEBUG("added '%s' usage text", cmd_name);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_help_add_command)
{
	if (CMD_ARGC < 2) {
		LOG_ERROR("%s: insufficient arguments", CMD_NAME);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* save help text and remove it from argument list */
	const char *str = CMD_ARGV[--CMD_ARGC];
	const char *help = !strcmp(CMD_NAME, "add_help_text") ? str : NULL;
	const char *usage = !strcmp(CMD_NAME, "add_usage_text") ? str : NULL;
	if (!help && !usage) {
		LOG_ERROR("command name '%s' is unknown", CMD_NAME);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	/* likewise for the leaf command name */
	const char *cmd_name = CMD_ARGV[--CMD_ARGC];

	struct command *c = NULL;
	if (CMD_ARGC > 0) {
		c = CMD_CTX->commands;
		int retval = CALL_COMMAND_HANDLER(command_help_find, c, &c);
		if (ERROR_OK != retval)
			return retval;
	}
	return help_add_command(CMD_CTX, c, cmd_name, help, usage);
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
	if (ERROR_OK != retval)
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
		.help = "Returns the command modes allowed by a  command:"
			"'any', 'config', or 'exec'.  If no command is"
			"specified, returns the current command mode.  "
			"Returns 'unknown' if an unknown command is given. "
			"Command can be multiple tokens.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration command_builtin_handlers[] = {
	{
		.name = "echo",
		.handler = jim_echo,
		.mode = COMMAND_ANY,
		.help = "Logs a message at \"user\" priority. "
			"Output message to stdout. "
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
	const char *HostOs;

	context->mode = COMMAND_EXEC;

	/* Create a jim interpreter if we were not handed one */
	if (interp == NULL) {
		/* Create an interpreter */
		interp = Jim_CreateInterp();
		/* Add all the Jim core commands */
		Jim_RegisterCoreCommands(interp);
		Jim_InitStaticExtensions(interp);
	}

	context->interp = interp;

	/* Stick to lowercase for HostOS strings. */
#if defined(_MSC_VER)
	/* WinXX - is generic, the forward
	 * looking problem is this:
	 *
	 *   "win32" or "win64"
	 *
	 * "winxx" is generic.
	 */
	HostOs = "winxx";
#elif defined(__linux__)
	HostOs = "linux";
#elif defined(__APPLE__) || defined(__DARWIN__)
	HostOs = "darwin";
#elif defined(__CYGWIN__)
	HostOs = "cygwin";
#elif defined(__MINGW32__)
	HostOs = "mingw32";
#elif defined(__ECOS)
	HostOs = "ecos";
#elif defined(__FreeBSD__)
	HostOs = "freebsd";
#elif defined(__NetBSD__)
	HostOs = "netbsd";
#elif defined(__OpenBSD__)
	HostOs = "openbsd";
#else
#warning "Unrecognized host OS..."
	HostOs = "other";
#endif
	Jim_SetGlobalVariableStr(interp, "ocd_HOSTOS",
		Jim_NewStringObj(interp, HostOs, strlen(HostOs)));

	Jim_CreateCommand(interp, "ocd_find", jim_find, NULL, NULL);
	Jim_CreateCommand(interp, "capture", jim_capture, NULL, NULL);

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
		if ((max == *ul) && (ERANGE == errno)) { \
			LOG_ERROR("Argument overflow");	\
			return ERROR_COMMAND_ARGUMENT_OVERFLOW;	\
		} \
		if (min && (min == *ul) && (ERANGE == errno)) { \
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
		if (ERROR_OK != retval)	\
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
