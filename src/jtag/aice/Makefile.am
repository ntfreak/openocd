noinst_LTLIBRARIES += %D%/libocdaice.la

%C%_libocdaice_la_CPPFLAGS = -I$(top_srcdir)/src/jtag/drivers $(AM_CPPFLAGS) $(LIBUSB1_CFLAGS)
%C%_libocdaice_la_SOURCES = \
	%D%/aice_transport.c \
	%D%/aice_interface.c \
	%D%/aice_port.c \
	%D%/aice_usb.c \
	%D%/aice_pipe.c \
	%D%/aice_transport.h \
	%D%/aice_interface.h \
	%D%/aice_port.h \
	%D%/aice_usb.h \
	%D%/aice_pipe.h
