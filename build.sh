#!/bin/bash -e
# -e=stop on any error, +e=continue on any error

set +x		# -x=debug on, +x=debug off

#
# Shell Variables, if one is not set then set a default value
#
# CPUS                  : Number of CPUS to use in a compile
# CROSS_COMPILE         : The root name of the cross compiler
# DEPLOY_PATH           : Where to place the build output
# LINUX_VERSION         : Linux version to compile
# PARENT_DIR            : The directory directly above in the build tree
# GRANDPARENT_DIR       : The directory two levels above in the build tree
# ROOTFS_PATH		: Location to build Linux modules at
# TARGET_MACH           : The default kernel image to construct

# count number of CPUS for make
if [ -z "$CPUS" ]
then
  export CPUS=$[$(echo /sys/devices/system/cpu/cpu[0-9]* | wc -w)+0]
  [ "$CPUS" -lt 1 ] && export CPUS=1
  echo "*** Using CPUS="$CPUS
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

# check for TARGET_MACH, use default if none provided
if [ "x$TARGET_MACH" == "x" ]; then
	export TARGET_MACH=nxp4330_bogota_defconfig
fi
echo "*** Using TARGET_MACH=$TARGET_MACH ***"

# check for CROSS_COMPILE, use default if none provided
if [ "x$CROSS_COMPILE" == "x" ]; then
        export CROSS_COMPILE=arm-angstrom-linux-uclibceabi-
        echo "CROSS_COMPILE not set, setting to $CROSS_COMPILE"
fi

# Base other directories relative to our grand parent dir
export PARENT_DIR=`dirname $PWD`
echo "*** Using PARENT_DIR=$PARENT_DIR ***"
export GRANDPARENT_DIR=`dirname $PARENT_DIR`
echo "*** Using GRANDPARENT_DIR=$GRANDPARENT_DIR ***"

if [ "x$DEPLOY_PATH" == "x" ]; then
        export DEPLOY_PATH=$GRANDPARENT_DIR/deploy
fi

echo "*** Using DEPLOY_PATH=$DEPLOY_PATH ***"

if ! [ -d $DEPLOY_PATH ]; then
        mkdir -p $DEPLOY_PATH
fi

if [ "x$ROOTFS_PATH" == "x" ]; then
	export ROOTFS_PATH=$DEPLOY_PATH/rootfs
fi

echo "*** Using ROOTFS_PATH=$ROOTFS_PATH ***"

if ! [ -d $ROOTFS_PATH ]; then
	mkdir -p $ROOTFS_PATH
fi

# Remove deploy binaries prior to build
rm -f $DEPLOY_PATH/Image-$LINUX_VERSION
rm -f $DEPLOY_PATH/uImage-$LINUX_VERSION
rm -f $DEPLOY_PATH/modules-$LINUX_VERSION.tar
if [ "$0" != "$(dirname $0)/build_surgeon.sh" ]; then
	rm -rf $ROOTFS_PATH
fi

if [ "x$LINUX_VERSION" == "x" ]; then
	LINUX_VERSION=3.4.39
fi

echo "$0: \$LINUX_VERSION='$LINUX_VERSION'"

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

if [ "$?" != "0" ]; then
	exit $?
fi

cp $(dirname $0)/arch/arm/boot/Image $DEPLOY_PATH/Image-$LINUX_VERSION
cp $(dirname $0)/arch/arm/boot/uImage $DEPLOY_PATH/uImage-$LINUX_VERSION

pushd $ROOTFS_PATH
tar -cvf $DEPLOY_PATH/modules-$LINUX_VERSION.tar ./lib/modules
popd

# compile finished sucesssfully
echo
echo "*** finished building Linux, target: $TARGET_MACH ***"
echo
