noinst_LTLIBRARIES += %D%/libhelper.la

%C%_libhelper_la_CPPFLAGS = $(AM_CPPFLAGS) $(LIBUSB1_CFLAGS)

%C%_libhelper_la_SOURCES = \
	%D%/binarybuffer.c \
	%D%/options.c \
	%D%/time_support_common.c \
	%D%/configuration.c \
	%D%/log.c \
	%D%/command.c \
	%D%/time_support.c \
	%D%/replacements.c \
	%D%/fileio.c \
	%D%/util.c \
	%D%/jep106.c \
	%D%/jim-nvp.c \
	%D%/binarybuffer.h \
	%D%/bits.h \
	%D%/configuration.h \
	%D%/list.h \
	%D%/util.h \
	%D%/types.h \
	%D%/log.h \
	%D%/command.h \
	%D%/time_support.h \
	%D%/replacements.h \
	%D%/fileio.h \
	%D%/system.h \
	%D%/jep106.h \
	%D%/jep106.inc \
	%D%/jim-nvp.h

%C%_libhelper_la_CFLAGS = $(AM_CFLAGS)
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
%C%_libhelper_la_CFLAGS += -Wno-sign-compare
endif

STARTUP_TCL_SRCS += %D%/startup.tcl
EXTRA_DIST += \
	%D%/bin2char.sh \
	%D%/update_jep106.pl
