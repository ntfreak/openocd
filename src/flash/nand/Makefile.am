noinst_LTLIBRARIES += %D%/libocdflashnand.la

%C%_libocdflashnand_la_SOURCES = \
	%D%/ecc.c \
	%D%/ecc_kw.c \
	%D%/core.c \
	%D%/fileio.c \
	%D%/tcl.c \
	%D%/arm_io.c \
	$(NAND_DRIVERS) \
	%D%/driver.c \
	$(NANDHEADERS)

NAND_DRIVERS = \
	%D%/nonce.c \
	%D%/davinci.c \
	%D%/lpc3180.c \
	%D%/lpc32xx.c \
	%D%/mxc.c \
	%D%/mx3.c \
	%D%/orion.c \
	%D%/s3c24xx.c \
	%D%/s3c2410.c \
	%D%/s3c2412.c \
	%D%/s3c2440.c \
	%D%/s3c2443.c \
	%D%/s3c6400.c \
	%D%/at91sam9.c \
	%D%/nuc910.c \
	%D%/jz4730.c

NANDHEADERS = \
	%D%/arm_io.h \
	%D%/core.h \
	%D%/driver.h \
	%D%/fileio.h \
	%D%/imp.h \
	%D%/lpc3180.h \
	%D%/lpc32xx.h \
	%D%/mxc.h \
	%D%/mx3.h \
	%D%/s3c24xx.h \
	%D%/s3c24xx_regs.h \
	%D%/nuc910.h
