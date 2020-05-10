#!/bin/bash -e

#cd linux-3.4.5
#cd linux-2.6.37/

TARGET_MACH=.config
echo "TARGET_MACH="$TARGET_MACH

# check for .config.  Use defconfig if it's not there.
if [ ! -e ".config" ]; then
	if  [ "x$TARGET_MACH" == "x" ]; then
		echo "*** Using default NXP3200 M2K form factor Board config file 'nxp3200_m2k_ubi_defconfig'."
		export TARGET_MACH=nxp4330_cabo_defconfig
	fi

        if [ ! -e "arch/arm/configs/$TARGET_MACH" ]; then
		echo "Machine configuration '$TARGET_MACH' not found, expected a filename from arch/arm/configs."
		exit 1
	fi
fi

make ARCH=arm KALLSYMS_EXTRA_PASS=1 -j $CPUS $TARGET_MACH
make ARCH=arm KALLSYMS_EXTRA_PASS=1 -j $CPUS uImage

MODS=`grep CONFIG_MODULES include/config/auto.conf`
if [ "$MODS" != "" ]; then
	echo "Making modules..."
	echo INSTALL_MOD_PATH=$MODULES_PATH
	make ARCH=arm -j $CPUS modules	
	make ARCH=arm -j $CPUS modules_install INSTALL_MOD_PATH=$MODULES_PATH
	cd $MODULES_PATH
	if [ -e "modules.tar.gz" ]; then
		echo "Removing old modules..."
		rm modules.tar.gz
	fi
	tar -zvcf modules.tar.gz lib/
fi
cd ../



