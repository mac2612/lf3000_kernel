#!/bin/bash -e

set +x

if [ "x$LINUX_VERSION" == "x" ]; then
	LINUX_VERSION=$1
fi

if [ "x$LINUX_VERSION" == "x" ]; then
	echo "\$LINUX_VERSION not set"
	exit 1
fi

echo "$0: \$LINUX_VERSION='$LINUX_VERSION'"

if [ ! -e $(dirname $0)/"linux-$LINUX_VERSION" ]; then
	echo "Directory linux-$LINUX_VERSION not found"
	exit 1
fi

# check if we can use sparse
which sparse > /dev/null
if [ "$?" = "0" ]; then
	SPARSE_CHECK="C=1"
else
	echo "**** WARNING: you do not have sparse installed"
	echo "**** run: \"sudo apt-get install sparse\" to install it"
	echo ""
	SPARSE_CHECK=""
fi

# count number of CPUs for make

if [ -z "$CPUS" ]
then
  export CPUS=`echo /sys/devices/system/cpu/cpu[0-9]* | wc -w`
  [ "$CPUS" -lt 1 ] && export CPUS=1
fi

pushd $(dirname $0)/linux-$LINUX_VERSION/

# check for .config.  Use defconfig if it's not there.
if [ ! -e ".config" ]; then
	if  [ "x$TARGET_MACH" == "x" ]; then
		export TARGET_MACH=nxp4330_vtk_lcd_defconfig
		echo "*** Using default NXP43300 VTK form factor Board config file '$TARGET_MACH'."
	fi

        if [ ! -e "arch/arm/configs/$TARGET_MACH" ]; then
		echo "Machine configuration '$TARGET_MACH' not found, expected a filename from arch/arm/configs."
		exit 1
	fi
	echo "TARGET_MACH="$TARGET_MACH
	make ARCH=arm -j $CPUS $TARGET_MACH
fi

echo "TARGET_MACH="$TARGET_MACH
make ARCH=arm -j $CPUS $SPARSE_CHECK KALLSYMS_EXTRA_PASS=1 uImage
make ARCH=arm -j $CPUS $SPARSE_CHECK KALLSYMS_EXTRA_PASS=1 zImage

MODS=`grep CONFIG_MODULES include/config/auto.conf`
if [ "$MODS" != "" ]; then
	make ARCH=arm -j $CPUS modules
	echo INSTALL_MOD_PATH=$ROOTFS_PATH
	make ARCH=arm -j $CPUS modules_install INSTALL_MOD_PATH=$ROOTFS_PATH
fi

popd
