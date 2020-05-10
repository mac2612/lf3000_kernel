#!/bin/sh

# Change these for your paths
export NXP4330_REPOSITORY=$HOME/git_workspaces
export E2K_ROOTFS_PATH=$HOME/nfsroot
#export PATH=$NXP4330_REPOSITORY/tools/arm-cortex_a9-eabi-4.6.3-eglibc-2.16/bin:$PATH
#export PATH=$NXP4330_REPOSITORY/tools/arm-2010.09/bin:$PATH
export PATH=/opt/angstrom-denzil/sysroots/i686-angstromsdk-linux/usr/bin/armv7a-vfp-angstrom-linux-uclibceabi:$PATH

#LF3K VTK 
#export TARGET_MACH=nxp4330_vtk_defconfig

#LF3K VTK with LCD
#export TARGET_MACH=nxp4330_vtk_lcd_defconfig

#LF3K R3K
#export TARGET_MACH=nxp4330_r3k_defconfig
#export TARGET_MACH=nxp4330_r3k_lcd_defconfig
#export TARGET_MACH=nxp4330_r3k_hdmi_defconfig
export TARGET_MACH=nxp4330_cabo_defconfig
#export TARGET_MACH=nxp4330_glasgow_defconfig
#export TARGET_MACH=nxp4330_r3k_nand_defconfig
#export TARGET_MACH=nxp4330_bogota_defconfig

#export CROSS_COMPILE=arm-cortex_a9-linux-gnueabi-
#export CROSS_COMPILE=arm-none-linux-gnueabi-
export CROSS_COMPILE=arm-angstrom-linux-uclibceabi-
export ARCH=arm
export LINUX_VERSION=3.4
export MODULES_PATH=$NXP4330_REPOSITORY/linux-$LINUX_VERSION/mymodules
export LOCALVERSION=

# count number of CPUs for make
if [ -z "$CPUS" ]
then
  export CPUS=`echo /sys/devices/system/cpu/cpu[0-9]* | wc -w`
  [ "$CPUS" -lt 1 ] && export CPUS=1
fi

#cd linux-3.4

make distclean
cp arch/arm/configs/$TARGET_MACH .config
make xconfig 

#cd ..



