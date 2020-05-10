#!/bin/bash -e

set +x		# -x=debug on, +x=debug off

#
#  Shell Variables, if one is not set then set a default value
#
#  LINUXDIST_REPOSITORY	 : Location of LinuxDist_LF2000 repository
#  SCRIPTS_PATH			 : Location of scripts in LINUXDIST_REPOSITORY
#  HOST_TOOLS_PATH		 : Location of host_tools in LINUXDIST_REPOSITORY
#  SURGEON_PATH			 : Where to place surgeon cbf/lfpkg build output
#
# FROM 'build.sh':
#  CPUS                  : Number of CPUS to use in a compile
#  CROSS_COMPILE         : The root name of the cross compiler
#  DEPLOY_PATH           : Where to place the build output
#  LINUX_VERSION         : Linux version to compile
#  PARENT_DIR            : The parent directory above in the build tree
#  ROOTFS_PATH			 : Location to build Linux modules at
#  TARGET_MACH           : The default kernel image to construct
. $(dirname $0)/build.sh

# check defconfig
if ! echo $TARGET_MACH | grep -c "surgeon" &> /dev/null; then 
	echo -e "\nError: not using a surgeon defconfig!\n"
	exit 1
fi

# check for LINUXDIST_REPOSITORY, use default if none provided
if [ "x$LINUXDIST_REPOSITORY" == "x" ]; then
	export LINUXDIST_REPOSITORY="$HOME/workspace/LinuxDist_LF2000"
fi
echo "*** Using LINUXDIST_REPOSITORY=$LINUXDIST_REPOSITORY ***"

# check for SCRIPTS_PATH, use default if none provided
if [ "x$SCRIPTS_PATH" == "x" ]; then
	export SCRIPTS_PATH="$LINUXDIST_REPOSITORY/scripts"
fi
echo "*** Using SCRIPTS_PATH=$SCRIPTS_PATH ***"

# check for HOST_TOOLS_PATH, use default if none provided
if [ "x$HOST_TOOLS_PATH" == "x" ]; then
	export HOST_TOOLS_PATH="$LINUXDIST_REPOSITORY/host_tools"
fi
echo "*** Using HOST_TOOLS_PATH=$HOST_TOOLS_PATH ***"

# check for SURGEON_PATH, use default if none provided
if [ "x$SURGEON_PATH" == "x" ]; then
	export SURGEON_PATH="$DEPLOY_PATH/Surgeon"
fi
echo "*** Using SURGEON_PATH=$SURGEON_PATH ***"

if ! [ -d $SURGEON_PATH ]; then
  mkdir -p $SURGEON_PATH
fi

######
# Do we want to use our custom initramfs with just build modules?
#	if yes, then let's create our custom initramfs.cpio.gz
#	if no, then continue...
if [ ! -z $1 ]; then
	if [ $1 == "-c" ]; then
		$(dirname $0)/initramfs.sh -c
		# Re-build with just built initramfs.cpio.gz to incorprate latest modules
		. $(dirname $0)/build.sh
	else
		echo "Ignoring invalid option: $1"
	fi
fi
######

# Remove lingering copies (disable bash stop on error around the following)
set +e
if [ -e $DEPLOY_PATH/Surgeon*lfp ]; then
	rm $DEPLOY_PATH/Surgeon*lfp
	rm $DEPLOY_PATH/Surgeon/*
fi
set -e

# Start concoction
echo -e "\nConcocting surgeon..."

# Create CBF
$SCRIPTS_PATH/make_cbf_pm_cabo.py -i $DEPLOY_PATH/Image-$LINUX_VERSION -o $SURGEON_PATH/surgeon.cbf

# Generate meta.inf for lfpkg
#	meta.inf content is for the most part irrelevant
VERSION="Unreleased"
cat > $SURGEON_PATH/meta.inf <<EOF
MetaVersion="1.0"
Device="LeapPadUltra"
Type="System"
ProductID=0x00270003
PackageID="PHR1-0x00270003-000000"
PartNumber="152-12352"
Version="$VERSION"
Locale="en-us"
Name="Surgeon"
ShortName="Surgeon"
Publisher="LeapFrog, Inc."
Developer="LeapFrog, Inc."
Hidden=1
BinFile="surgeon.cbf"
BuildDate="12/05/2013"
EOF

# Create LFP
pushd $(dirname $SURGEON_PATH)
$HOST_TOOLS_PATH/lfpkg -a create ./Surgeon
popd

# Deploy!
mv $DEPLOY_PATH/Surgeon-$VERSION.lfp $DEPLOY_PATH/Surgeon.lfp

# Surgeon is born!
echo
echo "*** finished building Surgeon, target: $TARGET_MACH ***"
echo
