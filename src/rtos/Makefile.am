noinst_LTLIBRARIES += %D%/librtos.la
%C%_librtos_la_SOURCES = \
	%D%/rtos.c \
	%D%/rtos_standard_stackings.c \
	%D%/rtos_ecos_stackings.c  \
	%D%/rtos_chibios_stackings.c \
	%D%/rtos_embkernel_stackings.c \
	%D%/rtos_mqx_stackings.c \
	%D%/rtos_ucos_iii_stackings.c \
	%D%/rtos_riot_stackings.c \
	%D%/FreeRTOS.c \
	%D%/ThreadX.c \
	%D%/eCos.c \
	%D%/linux.c \
	%D%/chibios.c \
	%D%/chromium-ec.c \
	%D%/embKernel.c \
	%D%/mqx.c \
	%D%/uCOS-III.c \
	%D%/nuttx.c \
	%D%/hwthread.c \
	%D%/riot.c \
	%D%/rtos.h \
	%D%/rtos_standard_stackings.h \
	%D%/rtos_ecos_stackings.h \
	%D%/linux_header.h \
	%D%/rtos_chibios_stackings.h \
	%D%/rtos_embkernel_stackings.h \
	%D%/rtos_mqx_stackings.h \
	%D%/rtos_riot_stackings.h \
	%D%/rtos_ucos_iii_stackings.h \
	%D%/nuttx_header.h

%C%_librtos_la_CFLAGS = $(AM_CFLAGS)

if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
%C%_librtos_la_CFLAGS += -Wno-sign-compare
endif
