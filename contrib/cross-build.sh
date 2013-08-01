#!/bin/sh
set -e -x

WORK_DIR=$PWD

## Source code paths, customize as necessary
: ${OPENOCD_SRC:="`dirname "$0"`/.."}
: ${LIBUSB1_SRC:=/path/to/libusb}

HOST_TRIPLET=$1
BUILD_DIR=$WORK_DIR/$HOST_TRIPLET-build
LIBUSB1_BUILD_DIR=$BUILD_DIR/libusb1
OPENOCD_BUILD_DIR=$BUILD_DIR/openocd

## Root of host file tree
SYSROOT=$WORK_DIR/$HOST_TRIPLET-root

## Install location within host file tree
: ${PREFIX=/usr}

## OpenOCD-only install dir for packaging
PACKAGE_DIR=$WORK_DIR/openocd_`git --git-dir=$OPENOCD_SRC/.git describe`_$HOST_TRIPLET

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
mkdir -p $LIBUSB1_BUILD_DIR
cd $LIBUSB1_BUILD_DIR
$LIBUSB1_SRC/configure --build=`$LIBUSB1_SRC/config.guess` --host=$HOST_TRIPLET \
--with-sysroot=$SYSROOT --prefix=$PREFIX \
$LIBUSB1_CONFIG
make
make install DESTDIR=$SYSROOT

# OpenOCD build & install into sysroot
mkdir -p $OPENOCD_BUILD_DIR
cd $OPENOCD_BUILD_DIR
$OPENOCD_SRC/configure --build=`$OPENOCD_SRC/config.guess` --host=$HOST_TRIPLET \
--with-sysroot=$SYSROOT --prefix=$PREFIX \
$OPENOCD_CONFIG
make
make install DESTDIR=$SYSROOT

# Separate OpenOCD install w/o dependencies. OpenOCD will have to be linked
# statically or have dependencies packaged/installed separately.
make install DESTDIR=$PACKAGE_DIR
