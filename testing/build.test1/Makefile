# -*- mode: makefile -*-
#
default: _complain_
include ./local.uses

%: _complain_


_complain_:
	@echo ""
	@echo " Try the target: cygwin.buildtest or linux.buildtest "
	@echo ""

remove.install:
	rm -rf ${INSTALL_DIR}

.PHONY: remove.install

cygwin.buildtest:
	${MAKE} -f Makefile.ftd2xx clean all
	${MAKE} -f Makefile.openocd cygwin.easy.permutations
	${MAKE} -f Makefile.openocd mingw32.easy.permutations
	${MAKE} -f Makefile.libftdi all
	${MAKE} -f Makefile.openocd cygwin.libftdi

linux.buildtest: 
	${MAKE} linux.easy.buildtest
	${MAKE} linux.ftd2xx_installed 
	${MAKE} linux.ft2232_libftdi
	@echo ""
	@echo ""
	@echo "========================================"
	@echo " Linux Build Tests Complete "
	@echo "========================================"
	@echo ""
	@echo ""


linux.easy.buildtest:
	@test -d openocd || (echo "Where the source to openocd?" && exit 1)
	${MAKE} -f Makefile.openocd bootstrap
	${MAKE} -f Makefile.ftd2xx  all
	${MAKE} -f Makefile.openocd linux.easy.permutations

linux.ftd2xx_installed: 
	${MAKE} remove.install
	${MAKE} linux.ftd2xx_installed.setup
	${MAKE} -f Makefile.openocd $@

 linux.ft2232_libftdi:
	${MAKE} remove.install 
	${MAKE} -f Makefile.libusb  all
	${MAKE} -f Makefile.confuse all
	${MAKE} -f Makefile.libftdi all
	${MAKE} -f Makefile.openocd $@

# This target is used to "install" files from
# the FTDICHIP.COM tar.gz unpack directory
# into "a proper place" - where they should be found.
linux.ftd2xx_installed.setup:
	mkdir -p ${INSTALL_DIR}/include
	mkdir -p ${EXEC_PREFIX}/lib
	@#
	@# Sanity check - make sure the .H file is findable
	@#
	@f=$(FTD2XX_LINUX_DIR)/ftd2xx.h && \
	test -f $$f || (echo "Error: $$f not found" ; exit 1)
	@#
	@# Header files are simple... just copy them.
	@#
	cp $(FTD2XX_LINUX_DIR)/ftd2xx.h   $(PREFIX)/include/.
	cp $(FTD2XX_LINUX_DIR)/WinTypes.h $(PREFIX)/include/.
	@#
	@# .SO files are harder.
	@#     (1) copy them, (2) make links
	@#
	cp $(FTD2XX_LINUX_DIR)/libftd2xx.so.$(FTD2XX_LINUX_VERSION) $(EXEC_PREFIX)/lib/.
	cd $(EXEC_PREFIX)/lib && rm -f libftd2xx.so.0
	cd $(EXEC_PREFIX)/lib && ln -s libftd2xx.so.$(FTD2XX_LINUX_VERSION) libftd2xx.so.0
	cd $(EXEC_PREFIX)/lib && rm -f libftd2xx.so
	cd $(EXEC_PREFIX)/lib && ln -s libftd2xx.so.$(FTD2XX_LINUX_VERSION) libftd2xx.so


all.download:
	mkdir -p ${VIRGINS}
	${MAKE} -f Makefile.confuse download
	${MAKE} -f Makefile.libftdi download
	${MAKE} -f Makefile.ftd2xx  download
	${MAKE} -f Makefile.libusb  download

.PHONY: linux.buildtest \
	linux.easy.buildtest \
	linux.ftd2xx_installed \
	linux.ft22232_libftdi \
	linux.ftd2xx_installed.setup
