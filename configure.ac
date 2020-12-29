AC_PREREQ([2.69])
AC_INIT([openocd], [0.11.0+dev],
  [OpenOCD Mailing List <openocd-devel@lists.sourceforge.net>])
AC_CONFIG_SRCDIR([src/openocd.c])
AC_CONFIG_AUX_DIR([.])

m4_include([config_subdir.m4])dnl

# check for makeinfo before calling AM_INIT_AUTOMAKE
AC_CHECK_PROG([MAKEINFO], [makeinfo], [makeinfo])
AS_IF([test "x$MAKEINFO" = "x"], [
  MAKEINFO='echo makeinfo missing; true'
  AC_MSG_WARN([Info documentation will not be built.])
])
AC_SUBST([MAKEINFO])

AM_INIT_AUTOMAKE([-Wall -Wno-portability dist-bzip2 dist-zip subdir-objects])

AC_CONFIG_HEADERS([config.h])
AH_BOTTOM([
#include <helper/system.h>
#include <helper/types.h>
#include <helper/replacements.h>
])

AC_LANG([C])
AC_PROG_CC
# autoconf 2.70 obsoletes AC_PROG_CC_C99 and includes it in AC_PROG_CC
m4_version_prereq([2.70],[],[AC_PROG_CC_C99])
AM_PROG_CC_C_O
AC_PROG_RANLIB
PKG_PROG_PKG_CONFIG([0.23])

dnl disable checks for C++, Fortran and GNU Java Compiler
m4_defun([_LT_AC_LANG_CXX_CONFIG], [:])
m4_defun([_LT_AC_LANG_F77_CONFIG], [:])
m4_defun([_LT_AC_LANG_GCJ_CONFIG], [:])
AC_DISABLE_SHARED
LT_INIT
AC_SUBST([LIBTOOL_DEPS])

dnl configure checks required for Jim files (these are obsolete w/ C99)
AC_C_CONST
AC_TYPE_LONG_LONG_INT

AC_SEARCH_LIBS([ioperm], [ioperm])
AC_SEARCH_LIBS([dlopen], [dl])
AC_SEARCH_LIBS([openpty], [util])

AC_CHECK_HEADERS([sys/socket.h])
AC_CHECK_HEADERS([elf.h])
AC_CHECK_HEADERS([dirent.h])
AC_CHECK_HEADERS([fcntl.h])
AC_CHECK_HEADERS([malloc.h])
AC_CHECK_HEADERS([netdb.h])
AC_CHECK_HEADERS([poll.h])
AC_CHECK_HEADERS([pthread.h])
AC_CHECK_HEADERS([strings.h])
AC_CHECK_HEADERS([sys/ioctl.h])
AC_CHECK_HEADERS([sys/param.h])
AC_CHECK_HEADERS([sys/select.h])
AC_CHECK_HEADERS([sys/stat.h])
AC_CHECK_HEADERS([sys/sysctl.h])
AC_CHECK_HEADERS([sys/time.h])
AC_CHECK_HEADERS([sys/types.h])
AC_CHECK_HEADERS([unistd.h])
AC_CHECK_HEADERS([arpa/inet.h ifaddrs.h netinet/in.h netinet/tcp.h net/if.h], [], [], [dnl
#include <stdio.h>
#ifdef STDC_HEADERS
# include <stdlib.h>
# include <stddef.h>
#else
# ifdef HAVE_STDLIB_H
#  include <stdlib.h>
# endif
#endif
#ifdef HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
])

AC_HEADER_ASSERT
AC_HEADER_STDBOOL

AC_C_BIGENDIAN

AC_CHECK_FUNCS([strndup])
AC_CHECK_FUNCS([strnlen])
AC_CHECK_FUNCS([gettimeofday])
AC_CHECK_FUNCS([usleep])
AC_CHECK_FUNCS([vasprintf])
AC_CHECK_FUNCS([realpath])

# guess-rev.sh only exists in the repository, not in the released archives
AC_MSG_CHECKING([whether to build a release])
AS_IF([test -x "$srcdir/guess-rev.sh"], [
  build_release=no
], [
  build_release=yes
])
AC_MSG_RESULT([$build_release])

# Adapter drivers
# 1st column -- configure option
# 2nd column -- description
# 3rd column -- symbol used for both config.h and automake
m4_define([ADAPTER_ARG], [m4_argn([1], $1)])
m4_define([ADAPTER_DESC], [m4_argn([2], $1)])
m4_define([ADAPTER_SYM], [m4_argn([3], $1)])
m4_define([ADAPTER_VAR], [enable_[]ADAPTER_ARG($1)])
m4_define([ADAPTER_OPT], [m4_translit(ADAPTER_ARG($1), [_], [-])])

m4_define([USB1_ADAPTERS],
	[[[ftdi], [MPSSE mode of FTDI based devices], [FTDI]],
	[[stlink], [ST-Link Programmer], [HLADAPTER_STLINK]],
	[[ti_icdi], [TI ICDI JTAG Programmer], [HLADAPTER_ICDI]],
	[[ulink], [Keil ULINK JTAG Programmer], [ULINK]],
	[[usb_blaster_2], [Altera USB-Blaster II Compatible], [USB_BLASTER_2]],
	[[ft232r], [Bitbang mode of FT232R based devices], [FT232R]],
	[[vsllink], [Versaloon-Link JTAG Programmer], [VSLLINK]],
	[[xds110], [TI XDS110 Debug Probe], [XDS110]],
	[[cmsis_dap_v2], [CMSIS-DAP v2 Compliant Debugger], [CMSIS_DAP_USB]],
	[[osbdm], [OSBDM (JTAG only) Programmer], [OSBDM]],
	[[opendous], [eStick/opendous JTAG Programmer], [OPENDOUS]],
	[[armjtagew], [Olimex ARM-JTAG-EW Programmer], [ARMJTAGEW]],
	[[rlink], [Raisonance RLink JTAG Programmer], [RLINK]],
	[[usbprog], [USBProg JTAG Programmer], [USBPROG]],
	[[aice], [Andes JTAG Programmer], [AICE]]])

m4_define([HIDAPI_ADAPTERS],
	[[[cmsis_dap], [CMSIS-DAP Compliant Debugger], [CMSIS_DAP_HID]],
	[[nulink], [Nu-Link Programmer], [HLADAPTER_NULINK]]])

m4_define([HIDAPI_USB1_ADAPTERS],
	[[[kitprog], [Cypress KitProg Programmer], [KITPROG]]])

m4_define([LIBFTDI_ADAPTERS],
	[[[usb_blaster], [Altera USB-Blaster Compatible], [USB_BLASTER]],
	[[presto], [ASIX Presto Adapter], [PRESTO]]])

m4_define([LIBFTDI_USB1_ADAPTERS],
	[[[openjtag], [OpenJTAG Adapter], [OPENJTAG]]])

m4_define([LIBGPIOD_ADAPTERS],
	[[[linuxgpiod], [Linux GPIO bitbang through libgpiod], [LINUXGPIOD]]])

m4_define([LIBJAYLINK_ADAPTERS],
	[[[jlink], [SEGGER J-Link Programmer], [JLINK]]])

m4_define([PCIE_ADAPTERS],
	[[[xlnx_pcie_xvc], [Xilinx XVC/PCIe], [XLNX_PCIE_XVC]]])

m4_define([OPTIONAL_LIBRARIES],
	[[[capstone], [Use Capstone disassembly framework], []]])

AC_ARG_ENABLE([doxygen-html],
  AS_HELP_STRING([--disable-doxygen-html],
    [Disable building Doxygen manual as HTML.]),
  [doxygen_as_html=$enableval], [doxygen_as_html=yes])
AC_SUBST([doxygen_as_html])
AC_MSG_CHECKING([whether to build Doxygen as HTML])
AC_MSG_RESULT([$doxygen_as_html])

AC_ARG_ENABLE([doxygen-pdf],
  AS_HELP_STRING([--enable-doxygen-pdf],
    [Enable building Doxygen manual as PDF.]),
  [doxygen_as_pdf=$enableval], [doxygen_as_pdf=no])
AC_SUBST([doxygen_as_pdf])
AC_MSG_CHECKING([whether to build Doxygen as PDF])
AC_MSG_RESULT([$doxygen_as_pdf])

AC_ARG_ENABLE([gccwarnings],
  AS_HELP_STRING([--disable-gccwarnings], [Disable compiler warnings]),
  [gcc_warnings=$enableval], [gcc_warnings=yes])

AC_ARG_ENABLE([wextra],
  AS_HELP_STRING([--disable-wextra], [Disable extra compiler warnings]),
  [gcc_wextra=$enableval], [gcc_wextra=$gcc_warnings])

AC_ARG_ENABLE([werror],
  AS_HELP_STRING([--disable-werror], [Do not treat warnings as errors]),
  [gcc_werror=$enableval], [gcc_werror=$gcc_warnings])

# set default verbose options, overridden by following options
debug_usb_io=no
debug_usb_comms=no

AC_ARG_ENABLE([verbose],
  AS_HELP_STRING([--enable-verbose],
      [Enable verbose JTAG I/O messages (for debugging).]),
  [
  debug_usb_io=$enableval
  debug_usb_comms=$enableval
  ], [])

AC_ARG_ENABLE([verbose_usb_io],
  AS_HELP_STRING([--enable-verbose-usb-io],
      [Enable verbose USB I/O messages (for debugging)]),
  [debug_usb_io=$enableval], [])

AC_ARG_ENABLE([verbose_usb_comms],
  AS_HELP_STRING([--enable-verbose-usb-comms],
      [Enable verbose USB communication messages (for debugging)]),
  [debug_usb_comms=$enableval], [])

AC_MSG_CHECKING([whether to enable verbose USB I/O messages]);
AC_MSG_RESULT([$debug_usb_io])
AS_IF([test "x$debug_usb_io" = "xyes"], [
  AC_DEFINE([_DEBUG_USB_IO_],[1], [Print verbose USB I/O messages])
])

AC_MSG_CHECKING([whether to enable verbose USB communication messages]);
AC_MSG_RESULT([$debug_usb_comms])
AS_IF([test "x$debug_usb_comms" = "xyes"], [
  AC_DEFINE([_DEBUG_USB_COMMS_],[1], [Print verbose USB communication messages])
])

debug_malloc=no
AC_ARG_ENABLE([malloc_logging],
  AS_HELP_STRING([--enable-malloc-logging],
      [Include free space in logging messages (requires malloc.h).]),
  [debug_malloc=$enableval], [])

AC_MSG_CHECKING([whether to enable malloc free space logging]);
AC_MSG_RESULT([$debug_malloc])
AS_IF([test "x$debug_malloc" = "xyes"], [
  AC_DEFINE([_DEBUG_FREE_SPACE_],[1], [Include malloc free space in logging])
])

AC_ARG_ENABLE([dummy],
  AS_HELP_STRING([--enable-dummy], [Enable building the dummy port driver]),
  [build_dummy=$enableval], [build_dummy=no])

AC_ARG_ENABLE([rshim],
  AS_HELP_STRING([--enable-rshim], [Enable building the rshim driver]),
  [build_rshim=$enableval], [build_rshim=no])

m4_define([AC_ARG_ADAPTERS], [
  m4_foreach([adapter], [$1],
	[AC_ARG_ENABLE(ADAPTER_OPT([adapter]),
		AS_HELP_STRING([--enable-ADAPTER_OPT([adapter])],
			[Enable building support for the ]ADAPTER_DESC([adapter])[ (default is $2)]),
		[], [ADAPTER_VAR([adapter])=$2])
  ])
])

AC_ARG_ADAPTERS([
  USB1_ADAPTERS,
  HIDAPI_ADAPTERS,
  HIDAPI_USB1_ADAPTERS,
  LIBFTDI_ADAPTERS,
  LIBFTDI_USB1_ADAPTERS
  LIBGPIOD_ADAPTERS,
  LIBJAYLINK_ADAPTERS
  ],[auto])

AC_ARG_ENABLE([parport],
  AS_HELP_STRING([--enable-parport], [Enable building the pc parallel port driver]),
  [build_parport=$enableval], [build_parport=no])

AC_ARG_ENABLE([parport_ppdev],
  AS_HELP_STRING([--disable-parport-ppdev],
      [Disable use of ppdev (/dev/parportN) for parport (for x86 only)]),
    [parport_use_ppdev=$enableval], [parport_use_ppdev=yes])

AC_ARG_ENABLE([parport_giveio],
    AS_HELP_STRING([--enable-parport-giveio],
      [Enable use of giveio for parport (for CygWin only)]),
    [parport_use_giveio=$enableval], [parport_use_giveio=])

AC_ARG_ENABLE([jtag_vpi],
  AS_HELP_STRING([--enable-jtag_vpi], [Enable building support for JTAG VPI]),
  [build_jtag_vpi=$enableval], [build_jtag_vpi=no])

AC_ARG_ENABLE([jtag_dpi],
  AS_HELP_STRING([--enable-jtag_dpi], [Enable building support for JTAG DPI]),
  [build_jtag_dpi=$enableval], [build_jtag_dpi=no])

AC_ARG_ENABLE([amtjtagaccel],
  AS_HELP_STRING([--enable-amtjtagaccel], [Enable building the Amontec JTAG-Accelerator driver]),
  [build_amtjtagaccel=$enableval], [build_amtjtagaccel=no])

AS_CASE(["${host_cpu}"],
  [arm*|aarch64], [
    AC_ARG_ENABLE([bcm2835gpio],
      AS_HELP_STRING([--enable-bcm2835gpio], [Enable building support for bitbanging on BCM2835 (as found in Raspberry Pi)]),
      [build_bcm2835gpio=$enableval], [build_bcm2835gpio=no])
    AC_ARG_ENABLE([imx_gpio],
      AS_HELP_STRING([--enable-imx_gpio], [Enable building support for bitbanging on NXP IMX processors]),
      [build_imx_gpio=$enableval], [build_imx_gpio=no])
  ],
  [
    build_bcm2835gpio=no
    build_imx_gpio=no
])

AS_CASE(["${host_cpu}"],
  [arm*], [
    AC_ARG_ENABLE([ep93xx],
      AS_HELP_STRING([--enable-ep93xx], [Enable building support for EP93xx based SBCs]),
      [build_ep93xx=$enableval], [build_ep93xx=no])

    AC_ARG_ENABLE([at91rm9200],
      AS_HELP_STRING([--enable-at91rm9200], [Enable building support for AT91RM9200 based SBCs]),
      [build_at91rm9200=$enableval], [build_at91rm9200=no])
  ],
  [
    build_ep93xx=no
    build_at91rm9200=no
])

AC_ARG_ENABLE([gw16012],
  AS_HELP_STRING([--enable-gw16012], [Enable building support for the Gateworks GW16012 JTAG Programmer]),
  [build_gw16012=$enableval], [build_gw16012=no])

AC_ARG_ENABLE([buspirate],
  AS_HELP_STRING([--enable-buspirate], [Enable building support for the Buspirate]),
  [build_buspirate=$enableval], [build_buspirate=no])

AC_ARG_ENABLE([sysfsgpio],
  AS_HELP_STRING([--enable-sysfsgpio], [Enable building support for programming driven via sysfs gpios.]),
  [build_sysfsgpio=$enableval], [build_sysfsgpio=no])

AC_ARG_ENABLE([xlnx_pcie_xvc],
  AS_HELP_STRING([--enable-xlnx-pcie-xvc], [Enable building support for Xilinx XVC/PCIe.]),
  [build_xlnx_pcie_xvc=$enableval], [build_xlnx_pcie_xvc=no])

AS_CASE([$host_os],
  [linux*], [],
  [
    AS_IF([test "x$build_sysfsgpio" = "xyes"], [
      AC_MSG_ERROR([sysfsgpio is only available on linux])
    ])

    AS_IF([test "x$enable_linuxgpiod" = "xyes"], [
      AC_MSG_ERROR([linuxgpiod is only available on linux])
    ])

    AS_IF([test "x$build_xlnx_pcie_xvc" = "xyes"], [
      AC_MSG_ERROR([xlnx_pcie_xvc is only available on linux])
    ])

    AS_CASE([$host_os], [freebsd*], [],
    [
      AS_IF([test "x$build_rshim" = "xyes"], [
        AC_MSG_ERROR([build_rshim is only available on linux or freebsd])
      ])
    ])
])

AC_ARG_ENABLE([internal-jimtcl],
  AS_HELP_STRING([--disable-internal-jimtcl], [Disable building internal jimtcl]),
  [use_internal_jimtcl=$enableval], [use_internal_jimtcl=yes])

AC_ARG_ENABLE([internal-libjaylink],
  AS_HELP_STRING([--disable-internal-libjaylink],
  [Disable building internal libjaylink]),
  [use_internal_libjaylink=$enableval], [use_internal_libjaylink=yes])

AC_ARG_ENABLE([remote-bitbang],
  AS_HELP_STRING([--enable-remote-bitbang], [Enable building support for the Remote Bitbang jtag driver]),
  [build_remote_bitbang=$enableval], [build_remote_bitbang=no])

AS_CASE(["${host_cpu}"],
  [i?86|x86*], [],
  [
    AS_IF([test "x$parport_use_ppdev" = "xno"], [
      AC_MSG_WARN([--disable-parport-ppdev is not supported by the host CPU])
    ])
    parport_use_ppdev=yes
])

AS_CASE([$host],
  [*-cygwin*], [
    is_win32=yes
    parport_use_ppdev=no

    AC_COMPILE_IFELSE([AC_LANG_PROGRAM([], [[return __MINGW32__;]])],
      [is_mingw=yes],[is_mingw=no])
    AS_IF([test "x$is_mingw" = "xyes"], [
      AS_IF([test "x$parport_use_giveio" = "xno"], [
        AC_MSG_WARN([--disable-parport-giveio is not supported by MinGW32 hosts])
      ])
      parport_use_giveio=yes
      is_cygwin=no
    ], [
      is_cygwin=yes
      # sys/io.h needed under cygwin for parport access
      AS_IF([test "x$build_parport" = "xyes"], [
        AC_CHECK_HEADERS([sys/io.h],[],AC_MSG_ERROR([Please install the cygwin ioperm package]))
      ])
    ])
  ],
  [*-mingw* | *-msys*], [
    is_mingw=yes
    is_win32=yes
    parport_use_ppdev=no

    AS_IF([test "x$parport_use_giveio" = "xno"], [
      AC_MSG_WARN([--disable-parport-giveio is not supported by MinGW32 hosts])
    ])
    parport_use_giveio=yes

    AS_IF([test "x$build_buspirate" = "xyes"], [
      AC_MSG_ERROR([buspirate currently not supported by MinGW32 hosts])
    ])

    AC_SUBST([HOST_CPPFLAGS], [-D__USE_MINGW_ANSI_STDIO])
  ],
  [*darwin*], [
    is_darwin=yes

    AS_IF([test "x$parport_use_giveio" = "xyes"], [
      AC_MSG_WARN([--enable-parport-giveio cannot be used by Darwin hosts])
    ])
    parport_use_giveio=no
  ],
  [
    AS_IF([test "x$parport_use_giveio" = "xyes"], [
      AC_MSG_WARN([--enable-parport-giveio cannot be used by ]$host[ hosts])
    ])
    parport_use_giveio=no
])

AS_IF([test "x$is_cygwin" = "xyes"], [
    AC_DEFINE([IS_CYGWIN], [1], [1 if building for Cygwin.])
], [
    AC_DEFINE([IS_CYGWIN], [0], [0 if not building for Cygwin.])
])

AS_IF([test "x$is_mingw" = "xyes"], [
    AC_DEFINE([IS_MINGW], [1], [1 if building for Mingw.])
], [
    AC_DEFINE([IS_MINGW], [0], [0 if not building for Mingw.])
])

AS_IF([test "x$is_win32" = "xyes"], [
    AC_DEFINE([IS_WIN32], [1], [1 if building for Win32.])
], [
    AC_DEFINE([IS_WIN32], [0], [0 if not building for Win32.])
])

AS_IF([test "x$is_darwin" = "xyes"], [
    AC_DEFINE([IS_DARWIN], [1], [1 if building for Darwin.])
], [
    AC_DEFINE([IS_DARWIN], [0], [0 if not building for Darwin.])
])

AS_IF([test "x$build_parport" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_PARPORT], [1], [1 if you want parport.])
], [
  AC_DEFINE([BUILD_PARPORT], [0], [0 if you don't want parport.])
])

AS_IF([test "x$build_rshim" = "xyes"], [
  AC_DEFINE([BUILD_RSHIM], [1], [1 if you want to debug BlueField SoC via rshim.])
], [
  AC_DEFINE([BUILD_RSHIM], [0], [0 if you don't want to debug BlueField SoC via rshim.])
])

AS_IF([test "x$build_dummy" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_DUMMY], [1], [1 if you want dummy driver.])
], [
  AC_DEFINE([BUILD_DUMMY], [0], [0 if you don't want dummy driver.])
])

AS_IF([test "x$build_ep93xx" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_EP93XX], [1], [1 if you want ep93xx.])
], [
  AC_DEFINE([BUILD_EP93XX], [0], [0 if you don't want ep93xx.])
])

AS_IF([test "x$build_at91rm9200" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_AT91RM9200], [1], [1 if you want at91rm9200.])
], [
  AC_DEFINE([BUILD_AT91RM9200], [0], [0 if you don't want at91rm9200.])
])

AS_IF([test "x$build_bcm2835gpio" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_BCM2835GPIO], [1], [1 if you want bcm2835gpio.])
], [
  AC_DEFINE([BUILD_BCM2835GPIO], [0], [0 if you don't want bcm2835gpio.])
])

AS_IF([test "x$build_imx_gpio" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_IMX_GPIO], [1], [1 if you want imx_gpio.])
], [
  AC_DEFINE([BUILD_IMX_GPIO], [0], [0 if you don't want imx_gpio.])
])

AS_IF([test "x$parport_use_ppdev" = "xyes"], [
  AC_DEFINE([PARPORT_USE_PPDEV], [1], [1 if you want parport to use ppdev.])
], [
  AC_DEFINE([PARPORT_USE_PPDEV], [0], [0 if you don't want parport to use ppdev.])
])

AS_IF([test "x$parport_use_giveio" = "xyes"], [
  AC_DEFINE([PARPORT_USE_GIVEIO], [1], [1 if you want parport to use giveio.])
], [
  AC_DEFINE([PARPORT_USE_GIVEIO], [0], [0 if you don't want parport to use giveio.])
])

AS_IF([test "x$build_jtag_vpi" = "xyes"], [
  AC_DEFINE([BUILD_JTAG_VPI], [1], [1 if you want JTAG VPI.])
], [
  AC_DEFINE([BUILD_JTAG_VPI], [0], [0 if you don't want JTAG VPI.])
])

AS_IF([test "x$build_jtag_dpi" = "xyes"], [
  AC_DEFINE([BUILD_JTAG_DPI], [1], [1 if you want JTAG DPI.])
], [
  AC_DEFINE([BUILD_JTAG_DPI], [0], [0 if you don't want JTAG DPI.])
])


AS_IF([test "x$build_amtjtagaccel" = "xyes"], [
  AC_DEFINE([BUILD_AMTJTAGACCEL], [1], [1 if you want the Amontec JTAG-Accelerator driver.])
], [
  AC_DEFINE([BUILD_AMTJTAGACCEL], [0], [0 if you don't want the Amontec JTAG-Accelerator driver.])
])

AS_IF([test "x$build_gw16012" = "xyes"], [
  AC_DEFINE([BUILD_GW16012], [1], [1 if you want the Gateworks GW16012 driver.])
], [
  AC_DEFINE([BUILD_GW16012], [0], [0 if you don't want the Gateworks GW16012 driver.])
])

AS_IF([test "x$build_buspirate" = "xyes"], [
  AC_DEFINE([BUILD_BUSPIRATE], [1], [1 if you want the Buspirate JTAG driver.])
], [
  AC_DEFINE([BUILD_BUSPIRATE], [0], [0 if you don't want the Buspirate JTAG driver.])
])

AS_IF([test "x$use_internal_jimtcl" = "xyes"], [
  AS_IF([test -f "$srcdir/jimtcl/configure.ac"], [
    AX_CONFIG_SUBDIR_OPTION([jimtcl], [--disable-install-jim])
  ], [
    AC_MSG_ERROR([jimtcl not found, run git submodule init and git submodule update.])
  ])
])

AS_IF([test "x$build_remote_bitbang" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_REMOTE_BITBANG], [1], [1 if you want the Remote Bitbang JTAG driver.])
], [
  AC_DEFINE([BUILD_REMOTE_BITBANG], [0], [0 if you don't want the Remote Bitbang JTAG driver.])
])

AS_IF([test "x$build_sysfsgpio" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_SYSFSGPIO], [1], [1 if you want the SysfsGPIO driver.])
], [
  AC_DEFINE([BUILD_SYSFSGPIO], [0], [0 if you don't want SysfsGPIO driver.])
])

AS_IF([test "x$build_xlnx_pcie_xvc" = "xyes"], [
  build_xlnx_pcie_xvc=yes
  AC_DEFINE([BUILD_XLNX_PCIE_XVC], [1], [1 if you want the Xilinx XVC/PCIe driver.])
], [
  AC_DEFINE([BUILD_XLNX_PCIE_XVC], [0], [0 if you don't want Xilinx XVC/PCIe driver.])
])

PKG_CHECK_MODULES([LIBUSB1], [libusb-1.0], [
	use_libusb1=yes
	AC_DEFINE([HAVE_LIBUSB1], [1], [Define if you have libusb-1.x])
	PKG_CHECK_EXISTS([libusb-1.0 >= 1.0.9],
		[AC_DEFINE([HAVE_LIBUSB_ERROR_NAME], [1], [Define if your libusb has libusb_error_name()])],
		[AC_MSG_WARN([libusb-1.x older than 1.0.9 detected, consider updating])])
	LIBUSB1_CFLAGS=`echo $LIBUSB1_CFLAGS | sed 's/-I/-isystem /'`
	AC_MSG_NOTICE([libusb-1.0 header bug workaround: LIBUSB1_CFLAGS changed to "$LIBUSB1_CFLAGS"])
	PKG_CHECK_EXISTS([libusb-1.0 >= 1.0.16],
		[AC_DEFINE([HAVE_LIBUSB_GET_PORT_NUMBERS], [1], [Define if your libusb has libusb_get_port_numbers()])])
  ], [
	use_libusb1=no
	AC_MSG_WARN([libusb-1.x not found, trying legacy libusb-0.1 as a fallback; consider installing libusb-1.x instead])
])

AC_ARG_WITH([capstone],
		AS_HELP_STRING([--with-capstone], [Use Capstone disassembly library (default=auto)])
	, [
		enable_capstone=$withval
	], [
		enable_capstone=auto
])

AS_IF([test "x$enable_capstone" != xno], [
	PKG_CHECK_MODULES([CAPSTONE], [capstone], [
		AC_DEFINE([HAVE_CAPSTONE], [1], [1 if you have Capstone disassembly framework.])
	], [
		AC_DEFINE([HAVE_CAPSTONE], [0], [0 if you don't have Capstone disassembly framework.])
		if test "x$enable_capstone" != xauto; then
			AC_MSG_ERROR([--with-capstone was given, but test for Capstone failed])
		fi
		enable_capstone=no
	])
])

for hidapi_lib in hidapi hidapi-hidraw hidapi-libusb; do
	PKG_CHECK_MODULES([HIDAPI],[$hidapi_lib],[
		use_hidapi=yes
		break
	],[
		use_hidapi=no
	])
done

PKG_CHECK_MODULES([LIBFTDI], [libftdi1], [
	use_libftdi=yes
	PKG_CHECK_EXISTS([libftdi1 >= 1.5],
		[AC_DEFINE([HAVE_LIBFTDI_TCIOFLUSH], [1], [Define if your libftdi has ftdi_tcioflush()])])
  ], [
	PKG_CHECK_MODULES([LIBFTDI], [libftdi], [use_libftdi=yes], [use_libftdi=no])
])

PKG_CHECK_MODULES([LIBGPIOD], [libgpiod], [use_libgpiod=yes], [use_libgpiod=no])

PKG_CHECK_MODULES([LIBJAYLINK], [libjaylink >= 0.2],
	[use_libjaylink=yes], [use_libjaylink=no])

m4_define([PROCESS_ADAPTERS], [
  m4_foreach([adapter], [$1], [
	AS_IF([test $2], [
		AS_IF([test "x$ADAPTER_VAR([adapter])" != "xno"], [
			AC_DEFINE([BUILD_]ADAPTER_SYM([adapter]), [1], [1 if you want the ]ADAPTER_DESC([adapter]).)
		], [
			AC_DEFINE([BUILD_]ADAPTER_SYM([adapter]), [0], [0 if you do not want the ]ADAPTER_DESC([adapter]).)
		])
	], [
		AS_IF([test "x$ADAPTER_VAR([adapter])" = "xyes"], [
			AC_MSG_ERROR([$3 is required for the ADAPTER_DESC([adapter])])
		])
		ADAPTER_VAR([adapter])=no
	])
	AM_CONDITIONAL(ADAPTER_SYM([adapter]), [test "x$ADAPTER_VAR([adapter])" != "xno"])
  ])
])

PROCESS_ADAPTERS([USB1_ADAPTERS], ["x$use_libusb1" = "xyes"], [libusb-1.x])
PROCESS_ADAPTERS([HIDAPI_ADAPTERS], ["x$use_hidapi" = "xyes"], [hidapi])
PROCESS_ADAPTERS([HIDAPI_USB1_ADAPTERS], ["x$use_hidapi" = "xyes" -a "x$use_libusb1" = "xyes"], [hidapi and libusb-1.x])
PROCESS_ADAPTERS([LIBFTDI_ADAPTERS], ["x$use_libftdi" = "xyes"], [libftdi])
PROCESS_ADAPTERS([LIBFTDI_USB1_ADAPTERS], ["x$use_libftdi" = "xyes" -a "x$use_libusb1" = "xyes"], [libftdi and libusb-1.x])
PROCESS_ADAPTERS([LIBGPIOD_ADAPTERS], ["x$use_libgpiod" = "xyes"], [libgpiod])
PROCESS_ADAPTERS([LIBJAYLINK_ADAPTERS], ["x$use_internal_libjaylink" = "xyes" -o "x$use_libjaylink" = "xyes"], [libjaylink-0.2])

AS_IF([test "x$enable_linuxgpiod" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_stlink" != "xno" -o "x$enable_ti_icdi" != "xno" -o "x$enable_nulink" != "xno"], [
	AC_DEFINE([BUILD_HLADAPTER], [1], [1 if you want the High Level JTAG driver.])
	AM_CONDITIONAL([HLADAPTER], [true])
], [
	AC_DEFINE([BUILD_HLADAPTER], [0], [0 if you want the High Level JTAG driver.])
	AM_CONDITIONAL([HLADAPTER], [false])
])
AM_CONDITIONAL([HLADAPTER_STLINK], [test "x$enable_stlink" != "xno"])
AM_CONDITIONAL([HLADAPTER_ICDI], [test "x$enable_ti_icdi" != "xno"])
AM_CONDITIONAL([HLADAPTER_NULINK], [test "x$enable_nulink" != "xno"])

AS_IF([test "x$enable_jlink" != "xno"], [
  AS_IF([test "x$use_internal_libjaylink" = "xyes"], [
    AS_IF([test -f "$srcdir/src/jtag/drivers/libjaylink/configure.ac"], [
      AX_CONFIG_SUBDIR_OPTION([src/jtag/drivers/libjaylink],
		[--enable-subproject-build])
    ], [
      AC_MSG_ERROR([Internal libjaylink not found, run either 'git submodule init' and 'git submodule update' or disable internal libjaylink with --disable-internal-libjaylink.])
    ])
  ])
])

# Presto needs the bitq module
AS_IF([test "x$enable_presto" != "xno"], [
  build_bitq=yes
])

AM_CONDITIONAL([RELEASE], [test "x$build_release" = "xyes"])
AM_CONDITIONAL([PARPORT], [test "x$build_parport" = "xyes"])
AM_CONDITIONAL([DUMMY], [test "x$build_dummy" = "xyes"])
AM_CONDITIONAL([GIVEIO], [test "x$parport_use_giveio" = "xyes"])
AM_CONDITIONAL([EP93XX], [test "x$build_ep93xx" = "xyes"])
AM_CONDITIONAL([AT91RM9200], [test "x$build_at91rm9200" = "xyes"])
AM_CONDITIONAL([BCM2835GPIO], [test "x$build_bcm2835gpio" = "xyes"])
AM_CONDITIONAL([IMX_GPIO], [test "x$build_imx_gpio" = "xyes"])
AM_CONDITIONAL([BITBANG], [test "x$build_bitbang" = "xyes"])
AM_CONDITIONAL([JTAG_VPI], [test "x$build_jtag_vpi" = "xyes" -o "x$build_jtag_vpi" = "xyes"])
AM_CONDITIONAL([JTAG_DPI], [test "x$build_jtag_dpi" = "xyes" -o "x$build_jtag_dpi" = "xyes"])
AM_CONDITIONAL([USB_BLASTER_DRIVER], [test "x$enable_usb_blaster" != "xno" -o "x$enable_usb_blaster_2" != "xno"])
AM_CONDITIONAL([AMTJTAGACCEL], [test "x$build_amtjtagaccel" = "xyes"])
AM_CONDITIONAL([GW16012], [test "x$build_gw16012" = "xyes"])
AM_CONDITIONAL([REMOTE_BITBANG], [test "x$build_remote_bitbang" = "xyes"])
AM_CONDITIONAL([BUSPIRATE], [test "x$build_buspirate" = "xyes"])
AM_CONDITIONAL([SYSFSGPIO], [test "x$build_sysfsgpio" = "xyes"])
AM_CONDITIONAL([XLNX_PCIE_XVC], [test "x$build_xlnx_pcie_xvc" = "xyes"])
AM_CONDITIONAL([USE_LIBUSB1], [test "x$use_libusb1" = "xyes"])
AM_CONDITIONAL([IS_CYGWIN], [test "x$is_cygwin" = "xyes"])
AM_CONDITIONAL([IS_MINGW], [test "x$is_mingw" = "xyes"])
AM_CONDITIONAL([IS_WIN32], [test "x$is_win32" = "xyes"])
AM_CONDITIONAL([IS_DARWIN], [test "x$is_darwin" = "xyes"])
AM_CONDITIONAL([BITQ], [test "x$build_bitq" = "xyes"])
AM_CONDITIONAL([USE_LIBFTDI], [test "x$use_libftdi" = "xyes"])
AM_CONDITIONAL([USE_LIBGPIOD], [test "x$use_libgpiod" = "xyes"])
AM_CONDITIONAL([USE_HIDAPI], [test "x$use_hidapi" = "xyes"])
AM_CONDITIONAL([USE_LIBJAYLINK], [test "x$use_libjaylink" = "xyes"])
AM_CONDITIONAL([RSHIM], [test "x$build_rshim" = "xyes"])
AM_CONDITIONAL([HAVE_CAPSTONE], [test "x$enable_capstone" != "xno"])

AM_CONDITIONAL([INTERNAL_JIMTCL], [test "x$use_internal_jimtcl" = "xyes"])
AM_CONDITIONAL([INTERNAL_LIBJAYLINK], [test "x$use_internal_libjaylink" = "xyes"])

# Look for environ alternatives.  Possibility #1: is environ in unistd.h or stdlib.h?
AC_MSG_CHECKING([for environ in unistd.h and stdlib.h])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#define _GNU_SOURCE
#include <unistd.h>
#include <stdlib.h>
  ]], [[char **ep = environ;]]
  )], [
    AC_MSG_RESULT([yes])
    has_environ=yes
  ], [
    AC_MSG_RESULT([no])

    # Possibility #2: can environ be found in an available library?
    AC_MSG_CHECKING([for extern environ])
    AC_LINK_IFELSE([AC_LANG_PROGRAM([[
    extern char **environ;
    ]], [[char **ep = environ;]]
      )], [
        AC_DEFINE(NEED_ENVIRON_EXTERN, [1], [Must declare 'environ' to use it.])
        has_environ=yes
      ], [
        has_environ=no
      ])
    AC_MSG_RESULT([${has_environ}])
  ])

AS_IF([test "x${has_environ}" != "xyes" ], [
  AC_MSG_FAILURE([Could not find 'environ' in unistd.h or available libraries.])
])

AC_DEFINE([_GNU_SOURCE],[1],[Use GNU C library extensions (e.g. stdndup).])

# set default gcc warnings
GCC_WARNINGS="-Wall -Wstrict-prototypes -Wformat-security -Wshadow"
AS_IF([test "x${gcc_wextra}" = "xyes"], [
  GCC_WARNINGS="${GCC_WARNINGS} -Wextra -Wno-unused-parameter"
  GCC_WARNINGS="${GCC_WARNINGS} -Wbad-function-cast"
  GCC_WARNINGS="${GCC_WARNINGS} -Wcast-align"
  GCC_WARNINGS="${GCC_WARNINGS} -Wredundant-decls"
  GCC_WARNINGS="${GCC_WARNINGS} -Wpointer-arith"
])
AS_IF([test "x${gcc_werror}" = "xyes"], [
  GCC_WARNINGS="${GCC_WARNINGS} -Werror"
])

# override default gcc cflags
AS_IF([test "x$gcc_warnings" = "xyes"], [
  AC_SUBST([GCC_WARNINGS], [$GCC_WARNINGS])
])

AC_SUBST(EXTRA_DIST_NEWS, ["$(echo $srcdir/NEWS-*)"])

AC_CONFIG_FILES([
  Makefile
])
AC_OUTPUT

echo
echo
echo OpenOCD configuration summary
echo --------------------------------------------------
m4_foreach([adapter], [USB1_ADAPTERS,
	HIDAPI_ADAPTERS, HIDAPI_USB1_ADAPTERS, LIBFTDI_ADAPTERS,
	LIBFTDI_USB1_ADAPTERS,
	LIBGPIOD_ADAPTERS,
	LIBJAYLINK_ADAPTERS, PCIE_ADAPTERS,
	OPTIONAL_LIBRARIES],
	[s=m4_format(["%-40s"], ADAPTER_DESC([adapter]))
	AS_CASE([$ADAPTER_VAR([adapter])],
		[auto], [
			echo "$s"yes '(auto)'
		],
		[yes], [
			echo "$s"yes
		],
		[no], [
			echo "$s"no
	])
])
echo
