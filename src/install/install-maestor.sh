#!/bin/bash
#
# This script installs ROS Fuerte with the
# openRAVE stacks.
#
# Options:
#   -y      : Quiet install. Answers yes to any prompts.
#   --auto-yes  : See -y
#
# Dependencies:
#   <Maestro install dir>/maestor/utils.sh
#   /etc/apt/sources.list    (If you're missing this, reinstall Ubuntu.)    
#   
# Blacklist:
#   /opt/ros/fuerte/stacks/armnavigation    (Mercury complains)
#   /opt/ros/fuerte/stacks/maestor      (No previous installs)
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

echo "Maestor installation Script"
echo "Version $VERSION"
echo ""

DEPENDENCY_DIRS="/etc /etc/apt /etc/apt/sources.list.d"
DEPENDENCY_FILES=""
BLACKLISTED_DIRS=""
BLACKLISTED_FILES=""

check dependency dir "$DEPENDENCY_DIRS"
if [[ $? != $SUCCESS ]]; then exit $NOT_FOUND; fi
check dependency file "$DEPENDENCY_FILES"
if [[ $? != $SUCCESS ]]; then exit $NOT_FOUND; fi
check blacklist dir "$BLACKLISTED_DIRS"
if [[ $? != $SUCCESS ]]; then exit $BLACKLIST_VIOLATED; fi
check blacklist file "$BLACKLISTED_FILES"
if [[ $? != $SUCCESS ]]; then exit $BLACKLIST_VIOLATED; fi

installDir=`pwd`

if [[ $# -lt 1 ]]; then
    QUIET=false
elif [[ "$1" == "-y" || "$1" == "--auto-yes" ]]; then
    QUIET=true
else
    QUIET=false
fi

cd /opt/ros/fuerte/stacks
source /opt/ros/fuerte/setup.bash
if [ $# -lt 1 ]
then
    echo ""
    echo ""
    echo "Would you like a link to your Maestor install in /opt/ros/fuerte/stacks?"
    select yn in "Yes" "No"; do
            case $yn in
                    Yes ) echo "Creating symbolic link in /opt/ros/fuerte/stacks..."; sudo ln -s $installDir/../ maestor; break;;
                    No ) echo "Skipping symbolic link creation..."; break;;
            esac
    done
    echo ""
elif [[ "$1" == "-y" ||  "$1" == "--auto-yes" ]]
then
    echo "Creating symbolic link in /opt/ros/fuerte/stacks..."; 
    sudo ln -s $installDir/../ maestor;
fi
cd /usr/bin
sudo ln -s $installDir/../run/maestor maestor 
rosmake maestor
echo "Installation complete."
