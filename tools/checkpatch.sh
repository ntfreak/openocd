#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

since=${1:-HEAD^}
tools/scripts/checkpatch.pl --git ${since}..
