#!/bin/bash
# version.sh: openocd version process automation
# Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>
# Release under the GNU GPL v2 (or later versions).

# FIXME Remove more bash-isms.  Fix errors making "ash -e" lose.

# NOTE Use with care!  "RC" should only follow x.x.x, with
# vendor tags after that.  Be traditional; avoid "rc0".

# NOTE:  This *ONLY* updates the "configure.ac" version tag.
# It does not affect GIT tags.  Use this script immediately
# before making a release, to remove the "-dev" tag and to
# update the version label.  Then commit the change and tag
# that commit to match the version label.

. "tools/release/helpers.sh"

do_version_usage() {
	cat << USAGE
usage: $0 <command>
Version Commands:
  tag {add|remove} <label>     Add or remove the specified tag.
  bump {major|minor|micro|rc}  Bump the specified version number, and
                               reset less-significant numbers to zero.
  bump tag <label>             Add or bump a versioned tag (e.g. -rcN).
  bump final <label>           Remove a versioned tag (e.g. -rcN).
USAGE
# REVISIT ... "commit" not listed.
}

do_version_sed() {
	local OLD_VERSION="${PACKAGE_VERSION}"
	local NEW_VERSION="$1"
	local MSG="$2"

	sed -i -e "/AC_INIT/ s|${OLD_VERSION}|${NEW_VERSION}|" configure.ac
	package_info_load
	echo "${MSG}: ${OLD_VERSION} -> ${NEW_VERSION}"
}
do_version_bump_sed() {
	local NEW_VERSION="$1"
	[ -z "${PACKAGE_VERSION_TAGS}" ] || \
		NEW_VERSION="${NEW_VERSION}${PACKAGE_VERSION_TAGS}"

	do_version_sed "${NEW_VERSION}" \
		"Bump ${CMD} package version number"
}
do_version_bump_major() {
	do_version_bump_sed "$((PACKAGE_MAJOR + 1)).0.0"
}
do_version_bump_minor() {
	do_version_bump_sed "${PACKAGE_MAJOR}.$((PACKAGE_MINOR + 1)).0"
}
do_version_bump_micro() {
	do_version_bump_sed "${PACKAGE_MAJOR_AND_MINOR}.$((PACKAGE_MICRO + 1))"
}
do_version_bump_tag() {
	local TAG="$1"
	[ "${TAG}" ] || die "TAG argument is missing"
	local TAGS="${PACKAGE_VERSION_TAGS}"
	if has_version_tag "${TAG}"; then
		local RC=$(do_version_tag_value "${TAG}")
		RC=$((${RC} + 1))
		TAGS=$(echo ${TAGS} | perl -npe "s/-${TAG}[\\d]*/-${TAG}${RC}/")
	else
		TAGS="-${TAG}0${PACKAGE_VERSION_TAGS}"
	fi
	PACKAGE_VERSION_TAGS="${TAGS}"
	do_version_bump_sed "${PACKAGE_VERSION_BASE}"
}
do_version_bump_final() {
	local TAG="$1"
	[ "${TAG}" ] || die "TAG argument is missing"
	has_version_tag "${TAG}" || die "-${TAG} tag is missing"
	do_version_tag_remove "${TAG}$(do_version_tag_value "${TAG}")"
}
do_version_bump() {
	CMD="$1"
	shift
	case "${CMD}" in
	major|minor|micro|final|tag)
		"do_version_bump_${CMD}" "$@"
		;;
	rc)
		do_version_bump_tag "rc"
		;;
	*)
		do_version_usage
		;;
	esac
}

has_version_tag() {
	test "${PACKAGE_VERSION/-${1}/}" != "${PACKAGE_VERSION}"
}
do_version_tag_value() {
	local TAG="$1"
	echo ${PACKAGE_VERSION_TAGS} | perl -ne "/-${TAG}"'(\d+)/ && print $1'
}
do_version_tag_add() {
	local TAG="$1"
	has_version_tag "${TAG}" && \
		die "error: tag '-${TAG}' exists in '${PACKAGE_VERSION}'"
	do_version_sed "${PACKAGE_VERSION}-${TAG}" \
		"Add '-${TAG}' version tag"
}
do_version_tag_remove() {
	local TAG="$1"
	has_version_tag "${TAG}" || \
		die "error: tag '-${TAG}' missing from '${PACKAGE_VERSION}'"
	do_version_sed "${PACKAGE_VERSION/-${TAG}/}" \
		"Remove '-${TAG}' version tag"
}
do_version_tag() {
	CMD="$1"
	shift
	case "${CMD}" in
	add|remove)
		local i=
		for i in "$@"; do
			"do_version_tag_${CMD}" "${i}"
		done
		;;
	*)
		do_version_usage
		;;
	esac
}

do_version() {
	CMD="$1"
	shift
	case "${CMD}" in
	tag|bump)
		"do_version_${CMD}" "$@"
		;;
	commit)
		do_version_commit "$@"
		;;
	*)
		do_version_usage
		;;
	esac
}

package_info_load
do_version "$@"
