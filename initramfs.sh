#!/bin/bash -e
# -e=stop on any error, +e=continue on any error

set +x		# -x=debug on, +x=debug off

## Action Variable
#    0 = no action (default)
#    1 = create
#    2 = extract
##
ACTION=0
EMPTY_OUT=0	# 0=nothing, 1=Empty extraction path first (no effect with creation)

# Usage / Help
usage () {
	echo -e "Usage: initramfs.sh [OPTION]"
	echo -e "  Options:"
	echo -e "\t -c <PATH_TO_ROOTFS> \t Create initramfs.cpio.gz"
	echo -e "\t -x <PATH_TO_CPIOGZ> \t Extract initramfs.cpio.gz"
	echo -e "\t -X \t\t\t Extract initramfs.cpio.gz (interactive)"
	echo -e "\t -e \t\t\t Empty extraction path"
	echo -e "\t -h \t\t\t Display this help"
	echo -e "\n  Notes:"
	echo -e "\t If no <PATH_TO_ROOTFS> is specified, then the used"
	echo -e "\t path is '\$NXP4330_REPOSITORY/deploy/rootfs/'."
	echo -e "\n\t When using the '-e' option, the extraction path"
	echo -e "\t used will be emptied prior to cpio.gz extraction."
	echo -e "\n"
	
	exit $1
}

# check if zenity is installed
if ( command -v zenity &> /dev/null ) then
    HAVE_ZENITY=1
else
    HAVE_ZENITY=0
fi

# Get Options
OPTIND=1
while getopts "cxXeh" opt; do
	case $opt in
		h)
			usage 1
			;;
		c)
			if [ $ACTION -gt 1 ]; then
				usage 1
			else
				ACTION=1
			fi
			;;
		e)
			EMPTY_OUT=1
			if [ $ACTION -eq 1 ]; then
			  echo "Option -$opt will do nothing..."
			fi
			;;
		x)
			if [ $ACTION -gt 0 ]; then
				usage 1
			else
				ACTION=2
  			XACTIVE=0
			fi
			;;
		X)
			if [ $ACTION -gt 0 ]; then
				usage 1
			else
				ACTION=2
  			XACTIVE=1
			fi
			;;
		\?) 
  			usage 1
  			;;
	esac
done

# Shift options to get 'PATH_TO_ROOTFS'/'PATH_TO_CPIOGZ'
shift $((OPTIND-1))


# check for NXP4330_REPOSITORY, use default if none provided
if [ "x$NXP4330_REPOSITORY" == "x" ]; then
	export NXP4330_REPOSITORY="$HOME/workspace/nxp4330.git"
fi

if [ "x$DEPLOY_PATH" == "x" ]; then
  DEPLOY_PATH="$NXP4330_REPOSITORY/deploy"
fi

if ! [ -d $DEPLOY_PATH ]; then
  mkdir -p $DEPLOY_PATH
fi

# check LINUX_VERSION, use default if none provided
if [ "x$LINUX_VERSION" == "x" ]; then
  LINUX_VERSION=3.4
fi

# set ROOTFS_PATH to be inside $DEPLOY_PATH
ROOTFS_PATH="$DEPLOY_PATH/rootfs"
if ! [ -d $ROOTFS_PATH ]; then
  mkdir -p $ROOTFS_PATH
fi
		
		
# Perform Action
case $ACTION in
	0)
		usage 1
		;;
	1)
	  # if argument is passed, use it for path to rootfs directory
	  if [ ! -z "$1" ]; then
	    ROOTFS_PATH="$1"
		fi
		
		if [ ! -d $ROOTFS_PATH ]; then
				echo "PATH_TO_ROOTFS not found!"
				usage 1
		fi

		# remove blacklisted files to avoid files sneaking from kernel build
		BL_COUNT=0
    while read LINE; do
      if ls $ROOTFS_PATH"$LINE" >& /dev/null; then
        BL_COUNT=$(expr $BL_COUNT + 1) # ((BL_COUNT++))
        echo -n "Found: \"$LINE\""
        rm $ROOTFS_PATH"$LINE" >& /dev/null
        echo " -- removed!"
      fi
    done < $ROOTFS_PATH/surgeon-blacklist.txt
    echo -e "Found $BL_COUNT blacklisted items\n"
		
		# Create gzipped cpio
		echo -e "\nCreating \"initramfs.cpio.gz\" in \"$(dirname $0)/linux-$LINUX_VERSION/\""
		sh -c "cd ${ROOTFS_PATH}/ && sudo find . | sudo cpio -H newc -o" | gzip -9 > "$(dirname $0)/initramfs.cpio.gz"
		;;
	2)
		# get path to .cpio.gz file
		if [ "$XACTIVE" -eq 1 ]; then
			CPIOGZ_PATH=$(zenity --file-selection --file-filter='Gzipped CPIO | *.cpio.gz' --title='Select .cpio.gz file to extract...')
	  else
	    CPIOGZ_PATH="$1"
		fi
		
		# check 'CPIOGZ_PATH' for *.cpio.gz existence
		if [ ! -e "$CPIOGZ_PATH" ] || ! echo "$CPIOGZ_PATH" | grep -c ".cpio.gz" &> /dev/null; then
			echo "PATH_TO_CPIOGZ: \"$CPIOGZ_PATH\" not found!"
			usage 1
		fi

		if [ $EMPTY_OUT == 1 ]; then
		  set +e  # continue if remove fails
			rm -r $ROOTFS_PATH/* &> /dev/null
			set -e  # re-set stop on any error
		fi

		echo "Extracting \"$(basename ${CPIOGZ_PATH})\" -> \"$ROOTFS_PATH\""
		gunzip -c ${CPIOGZ_PATH} | sudo sh -c "cd ${ROOTFS_PATH} && cpio -i"
		# Fix permissions
		USER=$(whoami)
		sudo chown -R ${USER}:${USER} ${ROOTFS_PATH}
		;;
esac

echo -e "\n$(basename $0): Done\n"
exit 0
