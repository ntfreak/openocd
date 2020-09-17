noinst_LTLIBRARIES += %D%/libopenocd.la
bin_PROGRAMS += %D%/openocd

%C%_openocd_SOURCES = \
	%D%/main.c

%C%_libopenocd_la_SOURCES = \
	%D%/hello.c %D%/hello.h \
	%D%/openocd.c %D%/openocd.h

%C%_openocd_LDADD = %D%/libopenocd.la

%C%_openocd_LDADD += $(MINGWLDADD)

if INTERNAL_JIMTCL
%C%_openocd_LDADD += $(top_builddir)/jimtcl/libjim.a
else
%C%_openocd_LDADD += -ljim
endif

%C%_libopenocd_la_CPPFLAGS =

# banner output includes RELSTR appended to $VERSION from the configure script
# guess-rev.sh returns either a repository version ID or "-snapshot"
if RELEASE
%C%_libopenocd_la_CPPFLAGS += -DRELSTR=\"\"
%C%_libopenocd_la_CPPFLAGS += -DGITVERSION=\"\"
else
%C%_libopenocd_la_CPPFLAGS += -DRELSTR=\"`$(top_srcdir)/guess-rev.sh $(top_srcdir)`\"
%C%_libopenocd_la_CPPFLAGS += -DGITVERSION=\"`cd $(top_srcdir) && git describe`\"
%C%_libopenocd_la_CPPFLAGS += -DPKGBLDDATE=\"`date +%F-%R`\"
endif

# add default CPPFLAGS
%C%_libopenocd_la_CPPFLAGS += $(AM_CPPFLAGS) $(CPPFLAGS)

# the library search path.
%C%_libopenocd_la_LDFLAGS = $(all_libraries)

if IS_MINGW
MINGWLDADD = -lws2_32
else
MINGWLDADD =
endif

%C%_libopenocd_la_LIBADD = \
	%D%/xsvf/libxsvf.la \
	%D%/svf/libsvf.la \
	%D%/pld/libpld.la \
	%D%/jtag/libjtag.la \
	%D%/transport/libtransport.la \
	%D%/flash/libflash.la \
	%D%/target/libtarget.la \
	%D%/server/libserver.la \
	%D%/rtos/librtos.la \
	%D%/helper/libhelper.la \
	%D%/rtt/librtt.la

BIN2C = $(srcdir)/%D%/helper/bin2char.sh

STARTUP_TCL_SRCS =
EXTRA_DIST += $(STARTUP_TCL_SRCS)

BUILT_SOURCES += %D%/startup_tcl.inc

# Convert .tcl to c-array
%D%/startup_tcl.inc: $(STARTUP_TCL_SRCS)
	cat $^ | $(BIN2C) > $@ || { rm -f $@; false; }

# add generated files to make clean list
CLEANFILES += %D%/startup_tcl.inc

# we do not want generated file in the dist
#dist-hook:
#	rm -f $(distdir)/%D%/startup_tcl.inc

include %D%/helper/Makefile.am
include %D%/jtag/Makefile.am
include %D%/transport/Makefile.am
include %D%/xsvf/Makefile.am
include %D%/svf/Makefile.am
include %D%/target/Makefile.am
include %D%/rtos/Makefile.am
include %D%/server/Makefile.am
include %D%/flash/Makefile.am
include %D%/pld/Makefile.am
include %D%/rtt/Makefile.am
