#!/bin/sh -e

die() {
	echo "$@" >&2
	exit 1
}

package_info_load_name() {
	grep AC_INIT configure.ac | perl -ne 's/^.+\(\[([-\w]*)\],.+$/$1/ and print'
}
package_info_load_version() {
	grep AC_INIT configure.ac | perl -ne 's/^.+\[([-\w\.]*)\],$/$1/ and print'
}

package_info_load() {
	[ -f "configure.ac" ] || \
		die "package_info_load: configure.ac is missing"

	PACKAGE_NAME="$(package_info_load_name)"
	# todo: fix this
	PACKAGE_TARNAME="${PACKAGE_NAME}"

	PACKAGE_VERSION="$(package_info_load_version)"

	[ "${PACKAGE_NAME}" -a "${PACKAGE_VERSION}" ] || \
		die "package information is missing from configure script"

	PACKAGE_VERSION_TAGS=
	[ "${PACKAGE_VERSION/-/}" = "${PACKAGE_VERSION}" ] || \
		PACKAGE_VERSION_TAGS="-${PACKAGE_VERSION#*-}"
	PACKAGE_VERSION_BASE="${PACKAGE_VERSION%%-*}"
	PACKAGE_MICRO="${PACKAGE_VERSION_BASE##*.}"
	PACKAGE_MAJOR_AND_MINOR="${PACKAGE_VERSION_BASE%.*}"
	PACKAGE_MAJOR="${PACKAGE_MAJOR_AND_MINOR%.*}"
	PACKAGE_MINOR="${PACKAGE_MAJOR_AND_MINOR#*.}"

	[ "${RELEASE_FINAL}" ] \
		&& RELEASE_VERSION="${PACKAGE_VERSION_BASE}" \
		|| RELEASE_VERSION="${PACKAGE_VERSION/-dev/}"
	PACKAGE_RELEASE="${PACKAGE_TARNAME}-${RELEASE_VERSION}"
	PACKAGE_STRING="${PACKAGE_NAME} ${PACKAGE_VERSION}"
}

package_info_show() {
	cat <<INFO
Name: ${PACKAGE_TARNAME}
Version: ${PACKAGE_VERSION}
Release: ${RELEASE_VERSION}
   Number: ${PACKAGE_VERSION_BASE}
   Series: ${PACKAGE_MAJOR_AND_MINOR}
    Major: ${PACKAGE_MAJOR}
    Minor: ${PACKAGE_MINOR}
    Micro: ${PACKAGE_MICRO}
     Tags: ${PACKAGE_VERSION_TAGS}
   Full: ${PACKAGE_TARNAME}-${PACKAGE_VERSION_BASE}${PACKAGE_VERSION_TAGS}
Release: ${PACKAGE_RELEASE}
   Type: ${RELEASE_TYPE}
INFO
}
