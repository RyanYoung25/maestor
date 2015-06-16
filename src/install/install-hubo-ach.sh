#!/bin/bash
#
# This script installs Hubo-Ach and the Hubo-Ach-Ros interface for use with
# various ROS stacks. Additionally, the hubo-ach-ros-visualization package
# will be installed and configured.
#
# Options: None
#
# Dependencies:
#	<Maestro Install Dir>/maestro/utils.sh
#
# Blacklist: None
#
# Author: Solis Knight
# Date: July 2013

# Change directory to the script's directory.
if [[ `echo "$0" | grep "/" | wc -l` > 0 ]]; then
    cd ${0%/*}
fi

# Source environment checking functions.
source ../utils.sh

# Stop execution on any significant error.
#set -e

echo "Hubo-ACH - Maestor installation script"
echo "Version $VERSION"
echo ""

DEPENDENCY_DIRS=""
DEPENDENCY_FILES=""
BLACKLISTED_DIRS=""
BLACKLISTED_FILES=""

check dependency dir "$DEPENDENCY_DIRS"
if [[ $? != 0 ]]; then exit $NOT_FOUND; fi
check dependency file "$DEPENDENCY_FILES"
if [[ $? != 0 ]]; then exit $NOT_FOUND; fi
check blacklist dir "$BLACKLISTED_DIRS"
if [[ $? != 0 ]]; then exit $BLACKLIST_VIOLATED; fi
check blacklist file "$BLACKLISTED_FILES"
if [[ $? != 0 ]]; then exit $BLACKLIST_VIOLATED; fi

if [[ $# > 0 && $1 == "-y" ]]; then
	QUIET=true
fi

if [[ -z "$QUIET" ]]; then
	read -p "Please enter installation directory (No trailing '/' Please): " installDir
else
	installDir=~/hubo-ach
fi

if [[ ! -d "$installDir" ]]; then
	mkdir $installDir
	if [[ ! -d "$installDir" ]]; then
		echo "Failed to create install directory."
		exit 1
	fi
fi

cd "$installDir"

add-apt-repository "deb http://code.golems.org/ubuntu precise golems.org"
#add-apt-repository "deb http://www.repo.danlofaro.com/release precise main"
apt-get update
apt-get install -y --force-yes --no-remove libach1 libach-dev ach-utils
apt-get install -y --force-yes libreadline-dev

git clone https://github.com/isaacgaretmia/hubo-ach

cd hubo-ach

git checkout power

apt-get install -y --force-yes --no-remove autoconf automake libtool autoconf-archive

autoreconf -i

./configure

make

make install


sudo ./first-time-install-hubo-ach.sh

#Hubo-Ach

source /opt/ros/indigo/setup.bash



