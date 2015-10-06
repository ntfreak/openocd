# ***************************************************************************
# *   Copyright (C) 2011 by Broadcom Corporation                            *
# *   Evan Hunter - ehunter@broadcom.com                                    *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU General Public License as published by  *
# *   the Free Software Foundation; either version 2 of the License, or     *
# *   (at your option) any later version.                                   *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU General Public License for more details.                          *
# *                                                                         *
# *   You should have received a copy of the GNU General Public License     *
# *   along with this program; if not, write to the                         *
# *   Free Software Foundation, Inc.,                                       *
# *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
# ***************************************************************************

include $(top_srcdir)/common.mk

METASOURCES = AUTO
noinst_LTLIBRARIES = librtos.la
noinst_HEADERS = rtos.h rtos_standard_stackings.h rtos_ecos_stackings.h linux_header.h rtos_chibios_stackings.h rtos_embkernel_stackings.h rtos_mqx_stackings.h rtos_riot_stackings.h
librtos_la_SOURCES = rtos.c rtos_standard_stackings.c rtos_ecos_stackings.c  rtos_chibios_stackings.c rtos_embkernel_stackings.c rtos_mqx_stackings.c rtos_riot_stackings.c FreeRTOS.c ThreadX.c eCos.c linux.c ChibiOS.c embKernel.c mqx.c riot.c

librtos_la_CFLAGS =
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
librtos_la_CFLAGS += -Wno-sign-compare
endif

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
