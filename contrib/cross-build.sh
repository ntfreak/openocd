#!/bin/sh

# This is an example of how to do a cross-build of OpenOCD using pkg-config.
# Cross-building with pkg-config is deceptively hard and most guides and
# tutorials are incomplete or give bad advice. Some of the traps that are easy
# to fall in but handled by this script are:
#
#  * Polluting search paths and flags with values from the build system.
#  * Faulty pkg-config wrappers shipped with distribution packaged cross-
#    toolchains.
#  * Build failing because pkg-config discards some paths even though they are
#    correctly listed in the .pc file.
#  * Getting successfully built binaries that cannot find runtime data because
#    paths refer to the build file system.
#
# This script is probably more useful as a reference than as a complete build
# tool but for some configurations it may be usable as-is. It only cross-builds
# libusb-1.0, hidapi, libftdi and capstone from source, but the script can be
# extended to build other prerequisites in a similar manner.
#
# Usage:
# export LIBUSB1_SRC=/path/to/libusb-1.0
# export HIDAPI_SRC=/path/to/hidapi
# export OPENOCD_CONFIG="--enable-..."
# cd /work/dir
# .../path/to/openocd/contrib/cross-build.sh <host-triplet>
#
# For static linking, a workaround is to
# export LIBUSB1_CONFIG="--enable-static --disable-shared"
#
# All the paths must not contain any spaces.

set -e -x

WORK_DIR=$PWD

## Source code paths, customize as necessary
: ${OPENOCD_SRC:="`dirname "$0"`/.."}
: ${LIBUSB1_SRC:=/path/to/libusb1}
: ${HIDAPI_SRC:=/path/to/hidapi}
: ${LIBFTDI_SRC:=/path/to/libftdi}
: ${CAPSTONE_SRC:=/path/to/capstone}

OPENOCD_SRC=`readlink -m $OPENOCD_SRC`
LIBUSB1_SRC=`readlink -m $LIBUSB1_SRC`
HIDAPI_SRC=`readlink -m $HIDAPI_SRC`
LIBFTDI_SRC=`readlink -m $LIBFTDI_SRC`
CAPSTONE_SRC=`readlink -m $CAPSTONE_SRC`

HOST_TRIPLET=$1
BUILD_DIR=$WORK_DIR/$HOST_TRIPLET-build
LIBUSB1_BUILD_DIR=$BUILD_DIR/libusb1
HIDAPI_BUILD_DIR=$BUILD_DIR/hidapi
LIBFTDI_BUILD_DIR=$BUILD_DIR/libftdi
CAPSTONE_BUILD_DIR=$BUILD_DIR/capstone
OPENOCD_BUILD_DIR=$BUILD_DIR/openocd

## Root of host file tree
SYSROOT=$WORK_DIR/$HOST_TRIPLET-root

## Install location within host file tree
: ${PREFIX=/usr}

## Make parallel jobs
: ${MAKE_JOBS:=1}

## OpenOCD-only install dir for packaging
: ${OPENOCD_TAG:=`git --git-dir=$OPENOCD_SRC/.git describe --tags`}
PACKAGE_DIR=$WORK_DIR/openocd_${OPENOCD_TAG}_${HOST_TRIPLET}

#######

# Create pkg-config wrapper and make sure it's used
export PKG_CONFIG=$WORK_DIR/$HOST_TRIPLET-pkg-config

cat > $PKG_CONFIG <<EOF
#!/bin/sh

SYSROOT=$SYSROOT

export PKG_CONFIG_DIR=
export PKG_CONFIG_LIBDIR=\${SYSROOT}$PREFIX/lib/pkgconfig:\${SYSROOT}$PREFIX/share/pkgconfig
export PKG_CONFIG_SYSROOT_DIR=\${SYSROOT}

# The following have to be set to avoid pkg-config to strip /usr/include and /usr/lib from paths
# before they are prepended with the sysroot path. Feels like a pkg-config bug.
export PKG_CONFIG_ALLOW_SYSTEM_CFLAGS=
export PKG_CONFIG_ALLOW_SYSTEM_LIBS=

exec pkg-config "\$@"
EOF
chmod +x $PKG_CONFIG

# Clear out work dir
rm -rf $SYSROOT $BUILD_DIR
mkdir -p $SYSROOT

# libusb-1.0 build & install into sysroot
if [ -d $LIBUSB1_SRC ] ; then
  mkdir -p $LIBUSB1_BUILD_DIR
  cd $LIBUSB1_BUILD_DIR
  $LIBUSB1_SRC/configure --build=`$LIBUSB1_SRC/config.guess` --host=$HOST_TRIPLET \
  --with-sysroot=$SYSROOT --prefix=$PREFIX \
  $LIBUSB1_CONFIG
  make -j $MAKE_JOBS
  make install DESTDIR=$SYSROOT
fi

# hidapi build & install into sysroot
if [ -d $HIDAPI_SRC ] ; then
  mkdir -p $HIDAPI_BUILD_DIR
  cd $HIDAPI_BUILD_DIR
  $HIDAPI_SRC/configure --build=`$HIDAPI_SRC/config.guess` --host=$HOST_TRIPLET \
    --with-sysroot=$SYSROOT --prefix=$PREFIX \
    $HIDAPI_CONFIG
  make -j $MAKE_JOBS
  make install DESTDIR=$SYSROOT
fi

# libftdi build & install into sysroot
if [ -d $LIBFTDI_SRC ] ; then
  mkdir -p $LIBFTDI_BUILD_DIR
  cd $LIBFTDI_BUILD_DIR
  # libftdi requires libusb1 static libraries, granted by:
  # export LIBUSB1_CONFIG="--enable-static ..."
  cmake $LIBFTDI_CONFIG \
    -DLIBUSB_INCLUDE_DIR=${SYSROOT}${PREFIX}/include/libusb-1.0 \
    -DLIBUSB_LIBRARIES=${SYSROOT}${PREFIX}/lib/libusb-1.0.a \
    -DCMAKE_INSTALL_PREFIX=${PREFIX} \
    -DPKG_CONFIG_EXECUTABLE=`which pkg-config` \
    $LIBFTDI_SRC
  make install DESTDIR=$SYSROOT
fi

# capstone build & install into sysroot
if [ -d $CAPSTONE_SRC ] ; then
  mkdir -p $CAPSTONE_BUILD_DIR
  cd $CAPSTONE_BUILD_DIR
  cp -r $CAPSTONE_SRC/* .
  make install DESTDIR=$SYSROOT PREFIX=$PREFIX \
    CROSS="${HOST_TRIPLET}-" \
    $CAPSTONE_CONFIG
  # fix the generated capstone.pc
  CAPSTONE_PC_FILE=${SYSROOT}${PREFIX}/lib/pkgconfig/capstone.pc
  sed -i '/^libdir=/d' $CAPSTONE_PC_FILE
  sed -i '/^includedir=/d' $CAPSTONE_PC_FILE
  sed -i '/^archive=/d' $CAPSTONE_PC_FILE
  sed -i '1s;^;prefix=/usr \
exec_prefix=${prefix} \
libdir=${exec_prefix}/lib \
includedir=${prefix}/include\n\n;' $CAPSTONE_PC_FILE
fi


# OpenOCD build & install into sysroot
mkdir -p $OPENOCD_BUILD_DIR
cd $OPENOCD_BUILD_DIR
$OPENOCD_SRC/configure --build=`$OPENOCD_SRC/config.guess` --host=$HOST_TRIPLET \
--with-sysroot=$SYSROOT --prefix=$PREFIX \
$OPENOCD_CONFIG
make -j $MAKE_JOBS
make install-strip DESTDIR=$SYSROOT

# Separate OpenOCD install w/o dependencies. OpenOCD will have to be linked
# statically or have dependencies packaged/installed separately.
make install-strip DESTDIR=$PACKAGE_DIR

