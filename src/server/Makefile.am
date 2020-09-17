noinst_LTLIBRARIES += %D%/libserver.la
%C%_libserver_la_SOURCES = \
	%D%/server.c \
	%D%/telnet_server.c \
	%D%/gdb_server.c \
	%D%/server.h \
	%D%/telnet_server.h \
	%D%/gdb_server.h \
	%D%/server_stubs.c \
	%D%/tcl_server.c \
	%D%/tcl_server.h \
	%D%/rtt_server.c \
	%D%/rtt_server.h

%C%_libserver_la_CFLAGS = $(AM_CFLAGS)
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
%C%_libserver_la_CFLAGS += -Wno-sign-compare
endif

STARTUP_TCL_SRCS += %D%/startup.tcl
