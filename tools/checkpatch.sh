#!/bin/sh
#

since=${1:-HEAD^}
git format-patch -M --stdout $since | \
    filterdiff -x "b/src/gnulib/*" | \
    tools/scripts/checkpatch.pl -
