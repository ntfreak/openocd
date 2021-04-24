/* Jim - A small embeddable Tcl interpreter
 *
 * Copyright 2005 Salvatore Sanfilippo <antirez@invece.org>
 * Copyright 2005 Clemens Hintze <c.hintze@gmx.net>
 * Copyright 2005 patthoyts - Pat Thoyts <patthoyts@users.sf.net>
 * Copyright 2008 oharboe - Øyvind Harboe - oyvind.harboe@zylin.com
 * Copyright 2008 Andrew Lunn <andrew@lunn.ch>
 * Copyright 2008 Duane Ellis <openocd@duaneellis.com>
 * Copyright 2008 Uwe Klein <uklein@klein-messgeraete.de>
 * Copyright 2008 Steve Bennett <steveb@workware.net.au>
 * Copyright 2009 Nico Coesel <ncoesel@dealogic.nl>
 * Copyright 2009 Zachary T Welch zw@superlucidity.net
 * Copyright 2009 David Brownell
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE JIM TCL PROJECT ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * JIM TCL PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation
 * are those of the authors and should not be interpreted as representing
 * official policies, either expressed or implied, of the Jim Tcl Project.
 */

#include <string.h>
#include <jim-nvp.h>

int jim_get_nvp(Jim_Interp *interp,
	Jim_Obj *objptr, const struct jim_nvp *nvp_table, const struct jim_nvp **result)
{
	struct jim_nvp *n;
	int e;

	e = jim_nvp_name2value_obj(interp, nvp_table, objptr, &n);
	if (e == JIM_ERR)
		return e;

	/* Success? found? */
	if (n->name) {
		/* remove const */
		*result = (struct jim_nvp *)n;
		return JIM_OK;
	} else
		return JIM_ERR;
}

struct jim_nvp *jim_nvp_name2value_simple(const struct jim_nvp *p, const char *name)
{
	while (p->name) {
		if (0 == strcmp(name, p->name))
			break;
		p++;
	}
	return (struct jim_nvp *)p;
}

struct jim_nvp *jim_nvp_name2value_nocase_simple(const struct jim_nvp *p, const char *name)
{
	while (p->name) {
		if (0 == strcasecmp(name, p->name))
			break;
		p++;
	}
	return (struct jim_nvp *)p;
}

int jim_nvp_name2value_obj(Jim_Interp *interp, const struct jim_nvp *p, Jim_Obj *o, struct jim_nvp **result)
{
	return jim_nvp_name2value(interp, p, Jim_String(o), result);
}

int jim_nvp_name2value(Jim_Interp *interp, const struct jim_nvp *_p, const char *name, struct jim_nvp **result)
{
	const struct jim_nvp *p;

	p = jim_nvp_name2value_simple(_p, name);

	/* result */
	if (result)
		*result = (struct jim_nvp *)p;

	/* found? */
	if (p->name)
		return JIM_OK;
	else
		return JIM_ERR;
}

int jim_nvp_name2value_obj_nocase(Jim_Interp *interp,
	const struct jim_nvp *p,
	Jim_Obj *o,
	struct jim_nvp **puthere)
{
	return jim_nvp_name2value_nocase(interp, p, Jim_String(o), puthere);
}

int jim_nvp_name2value_nocase(Jim_Interp *interp, const struct jim_nvp *_p, const char *name,
	struct jim_nvp **puthere)
{
	const struct jim_nvp *p;

	p = jim_nvp_name2value_nocase_simple(_p, name);

	if (puthere)
		*puthere = (struct jim_nvp *)p;
						/* found */
	if (p->name)
		return JIM_OK;
	else
		return JIM_ERR;
}

int jim_nvp_value2name_obj(Jim_Interp *interp, const struct jim_nvp *p, Jim_Obj *o, struct jim_nvp **result)
{
	int e;
	jim_wide w;

	e = Jim_GetWide(interp, o, &w);
	if (e != JIM_OK)
		return e;

	return jim_nvp_value2name(interp, p, w, result);
}

struct jim_nvp *jim_nvp_value2name_simple(const struct jim_nvp *p, int value)
{
	while (p->name) {
		if (value == p->value)
			break;
		p++;
	}
	return (struct jim_nvp *)p;
}

int jim_nvp_value2name(Jim_Interp *interp, const struct jim_nvp *_p, int value, struct jim_nvp **result)
{
	const struct jim_nvp *p;

	p = jim_nvp_value2name_simple(_p, value);

	if (result)
		*result = (struct jim_nvp *)p;

	if (p->name)
		return JIM_OK;
	else
		return JIM_ERR;
}

int jim_getopt_setup(struct jim_getopt_info *p, Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	memset(p, 0, sizeof(*p));
	p->interp = interp;
	p->argc = argc;
	p->argv = argv;

	return JIM_OK;
}

void jim_getopt_debug(struct jim_getopt_info *p)
{
	int x;

	fprintf(stderr, "---args---\n");
	for (x = 0; x < p->argc; x++)
		fprintf(stderr, "%2d) %s\n", x, Jim_String(p->argv[x]));
	fprintf(stderr, "-------\n");
}

int jim_getopt_obj(struct jim_getopt_info *goi, Jim_Obj **puthere)
{
	Jim_Obj *o;

	o = NULL;		/* failure */
	if (goi->argc) {
		/* success */
		o = goi->argv[0];
		goi->argc -= 1;
		goi->argv += 1;
	}
	if (puthere)
		*puthere = o;
	if (o != NULL)
		return JIM_OK;
	else
		return JIM_ERR;
}

int jim_getopt_string(struct jim_getopt_info *goi, const char **puthere, int *len)
{
	int r;
	Jim_Obj *o;
	const char *cp;

	r = jim_getopt_obj(goi, &o);
	if (r == JIM_OK) {
		cp = Jim_GetString(o, len);
		if (puthere) {
			*puthere = cp;
		}
	}
	return r;
}

int jim_getopt_double(struct jim_getopt_info *goi, double *puthere)
{
	int r;
	Jim_Obj *o;
	double _safe;

	if (puthere == NULL)
		puthere = &_safe;

	r = jim_getopt_obj(goi, &o);
	if (r == JIM_OK) {
		r = Jim_GetDouble(goi->interp, o, puthere);
		if (r != JIM_OK)
			Jim_SetResultFormatted(goi->interp, "not a number: %#s", o);
	}
	return r;
}

int jim_getopt_wide(struct jim_getopt_info *goi, jim_wide *puthere)
{
	int r;
	Jim_Obj *o;
	jim_wide _safe;

	if (puthere == NULL)
		puthere = &_safe;

	r = jim_getopt_obj(goi, &o);
	if (r == JIM_OK)
		r = Jim_GetWide(goi->interp, o, puthere);
	return r;
}

int jim_getopt_nvp(struct jim_getopt_info *goi, const struct jim_nvp *nvp, struct jim_nvp **puthere)
{
	struct jim_nvp *_safe;
	Jim_Obj *o;
	int e;

	if (puthere == NULL)
		puthere = &_safe;

	e = jim_getopt_obj(goi, &o);
	if (e == JIM_OK)
		e = jim_nvp_name2value_obj(goi->interp, nvp, o, puthere);

	return e;
}

void jim_getopt_nvp_unknown(struct jim_getopt_info *goi, const struct jim_nvp *nvptable, int hadprefix)
{
	if (hadprefix)
		jim_set_result_nvp_unknown(goi->interp, goi->argv[-2], goi->argv[-1], nvptable);
	else
		jim_set_result_nvp_unknown(goi->interp, NULL, goi->argv[-1], nvptable);
}

int jim_getopt_enum(struct jim_getopt_info *goi, const char *const *lookup, int *puthere)
{
	int _safe;
	Jim_Obj *o;
	int e;

	if (puthere == NULL)
		puthere = &_safe;
	e = jim_getopt_obj(goi, &o);
	if (e == JIM_OK)
		e = Jim_GetEnum(goi->interp, o, lookup, puthere, "option", JIM_ERRMSG);
	return e;
}

void jim_set_result_nvp_unknown(Jim_Interp *interp,
	Jim_Obj *param_name, Jim_Obj *param_value, const struct jim_nvp *nvp)
{
	if (param_name)
		Jim_SetResultFormatted(interp,
			"%#s: Unknown: %#s, try one of: ",
			param_name,
			param_value);
	else
		Jim_SetResultFormatted(interp, "Unknown param: %#s, try one of: ", param_value);
	while (nvp->name) {
		const char *a;
		const char *b;

		if ((nvp + 1)->name) {
			a = nvp->name;
			b = ", ";
		} else {
			a = "or ";
			b = nvp->name;
		}
		Jim_AppendStrings(interp, Jim_GetResult(interp), a, b, NULL);
		nvp++;
	}
}

const char *jim_debug_argv_string(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	static Jim_Obj *debug_string_obj;

	int x;

	if (debug_string_obj)
		Jim_FreeObj(interp, debug_string_obj);

	debug_string_obj = Jim_NewEmptyStringObj(interp);
	for (x = 0; x < argc; x++)
		Jim_AppendStrings(interp, debug_string_obj, Jim_String(argv[x]), " ", NULL);

	return Jim_String(debug_string_obj);
}
