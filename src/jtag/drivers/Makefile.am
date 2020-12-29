noinst_LTLIBRARIES += %D%/libocdjtagdrivers.la
%C%_libocdjtagdrivers_la_LIBADD =

%C%_libocdjtagdrivers_la_SOURCES = \
	$(DRIVERFILES) \
	$(DRIVERHEADERS)

%C%_libocdjtagdrivers_la_CPPFLAGS = $(AM_CPPFLAGS)

ULINK_FIRMWARE = %D%/OpenULINK

EXTRA_DIST += $(ULINK_FIRMWARE) \
	%D%/usb_blaster/README.CheapClone \
	%D%/Makefile.rlink \
	%D%/rlink_call.m4 \
	%D%/rlink_init.m4

DRIVERFILES =

# Standard Driver: common files
DRIVERFILES += %D%/driver.c
DRIVERFILES += %D%/jtag_usb_common.c

if USE_LIBUSB1
DRIVERFILES += %D%/libusb_helper.c
%C%_libocdjtagdrivers_la_CPPFLAGS += $(LIBUSB1_CFLAGS)
%C%_libocdjtagdrivers_la_LIBADD += $(LIBUSB1_LIBS)
endif

if USE_LIBFTDI
%C%_libocdjtagdrivers_la_CPPFLAGS += $(LIBFTDI_CFLAGS)
%C%_libocdjtagdrivers_la_LIBADD += $(LIBFTDI_LIBS)
endif

if USE_LIBGPIOD
%C%_libocdjtagdrivers_la_CPPFLAGS += $(LIBGPIOD_CFLAGS)
%C%_libocdjtagdrivers_la_LIBADD += $(LIBGPIOD_LIBS)
endif

if USE_HIDAPI
%C%_libocdjtagdrivers_la_CPPFLAGS += $(HIDAPI_CFLAGS)
%C%_libocdjtagdrivers_la_LIBADD += $(HIDAPI_LIBS)
endif

if USE_LIBJAYLINK
%C%_libocdjtagdrivers_la_CPPFLAGS += $(LIBJAYLINK_CFLAGS)
%C%_libocdjtagdrivers_la_LIBADD += $(LIBJAYLINK_LIBS)
endif

if JLINK
DRIVERFILES += %D%/jlink.c
if INTERNAL_LIBJAYLINK
SUBDIRS += %D%/libjaylink
DIST_SUBDIRS += %D%/libjaylink

%C%_libocdjtagdrivers_la_LIBADD += %D%/libjaylink/libjaylink/libjaylink.la
%C%_libocdjtagdrivers_la_CPPFLAGS += -I$(builddir)/%D%/libjaylink/libjaylink -I$(srcdir)/%D%/libjaylink
endif
endif

if BITBANG
DRIVERFILES += %D%/bitbang.c
endif
if PARPORT
DRIVERFILES += %D%/parport.c
endif
if DUMMY
DRIVERFILES += %D%/dummy.c
endif
if FTDI
DRIVERFILES += %D%/ftdi.c %D%/mpsse.c
endif
if LINUXGPIOD
DRIVERFILES += %D%/linuxgpiod.c
endif
if JTAG_VPI
DRIVERFILES += %D%/jtag_vpi.c
endif
if JTAG_DPI
DRIVERFILES += %D%/jtag_dpi.c
endif
if USB_BLASTER_DRIVER
%C%_libocdjtagdrivers_la_LIBADD += %D%/usb_blaster/libocdusbblaster.la
include %D%/usb_blaster/Makefile.am
endif
if FT232R
DRIVERFILES += %D%/ft232r.c
endif
if AMTJTAGACCEL
DRIVERFILES += %D%/amt_jtagaccel.c
endif
if EP93XX
DRIVERFILES += %D%/ep93xx.c
endif
if AT91RM9200
DRIVERFILES += %D%/at91rm9200.c
endif
if GW16012
DRIVERFILES += %D%/gw16012.c
endif
if BITQ
DRIVERFILES += %D%/bitq.c
endif
if PRESTO
DRIVERFILES += %D%/presto.c
endif
if USBPROG
DRIVERFILES += %D%/usbprog.c
endif
if RLINK
DRIVERFILES += %D%/rlink.c %D%/rlink_speed_table.c
endif
if ULINK
DRIVERFILES += %D%/ulink.c
ulinkdir = $(pkgdatadir)/OpenULINK
dist_ulink_DATA = $(ULINK_FIRMWARE)/ulink_firmware.hex
%C%_libocdjtagdrivers_la_LIBADD += -lm
endif
if VSLLINK
DRIVERFILES += %D%/versaloon/usbtoxxx/usbtogpio.c
DRIVERFILES += %D%/versaloon/usbtoxxx/usbtojtagraw.c
DRIVERFILES += %D%/versaloon/usbtoxxx/usbtoswd.c
DRIVERFILES += %D%/versaloon/usbtoxxx/usbtopwr.c
DRIVERFILES += %D%/versaloon/usbtoxxx/usbtoxxx.c
DRIVERFILES += %D%/versaloon/versaloon.c
DRIVERFILES += %D%/vsllink.c
endif
if ARMJTAGEW
DRIVERFILES += %D%/arm-jtag-ew.c
endif
if BUSPIRATE
DRIVERFILES += %D%/buspirate.c
endif
if REMOTE_BITBANG
DRIVERFILES += %D%/remote_bitbang.c
endif
if HLADAPTER_STLINK
DRIVERFILES += %D%/stlink_usb.c
endif
if HLADAPTER_ICDI
DRIVERFILES += %D%/ti_icdi_usb.c
endif
if HLADAPTER_NULINK
DRIVERFILES += %D%/nulink_usb.c
endif
if RSHIM
DRIVERFILES += %D%/rshim.c
endif
if OSBDM
DRIVERFILES += %D%/osbdm.c
endif
if OPENDOUS
DRIVERFILES += %D%/opendous.c
endif
if SYSFSGPIO
DRIVERFILES += %D%/sysfsgpio.c
endif
if XLNX_PCIE_XVC
DRIVERFILES += %D%/xlnx-pcie-xvc.c
endif
if BCM2835GPIO
DRIVERFILES += %D%/bcm2835gpio.c
endif
if OPENJTAG
DRIVERFILES += %D%/openjtag.c
endif
if CMSIS_DAP_HID
DRIVERFILES += %D%/cmsis_dap_usb_hid.c
DRIVERFILES += %D%/cmsis_dap.c
endif
if CMSIS_DAP_USB
DRIVERFILES += %D%/cmsis_dap_usb_bulk.c
if !CMSIS_DAP_HID
DRIVERFILES += %D%/cmsis_dap.c
endif
endif
if IMX_GPIO
DRIVERFILES += %D%/imx_gpio.c
endif
if KITPROG
DRIVERFILES += %D%/kitprog.c
endif
if XDS110
DRIVERFILES += %D%/xds110.c
endif

DRIVERHEADERS = \
	%D%/bitbang.h \
	%D%/bitq.h \
	%D%/jtag_usb_common.h \
	%D%/libftdi_helper.h \
	%D%/libusb_helper.h \
	%D%/cmsis_dap.h \
	%D%/minidriver_imp.h \
	%D%/mpsse.h \
	%D%/rlink.h \
	%D%/rlink_dtc_cmd.h \
	%D%/rlink_ep1_cmd.h \
	%D%/rlink_st7.h \
	%D%/versaloon/usbtoxxx/usbtoxxx.h \
	%D%/versaloon/usbtoxxx/usbtoxxx_internal.h \
	%D%/versaloon/versaloon.h \
	%D%/versaloon/versaloon_include.h \
	%D%/versaloon/versaloon_internal.h
