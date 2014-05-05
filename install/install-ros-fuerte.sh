#!/bin/bash
#
# This script installs ROS Fuerte with the Orocos Toolchain and
# openRAVE stacks.
#
# Options:
#   -y      : Quiet install. Answers yes to any prompts.
#   --auto-yes  : See -y
#
# Dependencies:
#   <Maestro install dir>/maestro/utils.sh
#   /etc/apt/sources.list    (If you're missing this, reinstall Ubuntu.)    
#   
# Blacklist:
#   /opt/ros/fuerte/stacks/armnavigation    (Mercury complains)
#   /opt/ros/fuerte/stacks/maestro      (No previous installs)
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

echo "ROS-Fuerte installation Script"
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

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
apt-get update
apt-get install -y --no-remove mercurial
apt-get install -y --no-remove ros-fuerte-desktop-full

if [[ `grep 'source /opt/ros/fuerte/setup.bash' ~/.bashrc | wc -l` == 0 && $QUIET == false ]]; then
    echo ""
    echo ""
    echo "Would you like to add a source line to your bashrc file?"
    select yn in "Yes" "No"; do
        case $yn in
        Yes ) 
            echo "Adding source command to bashrc..." 
            echo "source /opt/ros/fuerte/setup.bash" >> ~/.bashrc 
            break;;
        No ) 
            echo "Skipping modification of bashrc..."
            break;;
        esac
    done
    echo ""
elif $QUIET == true; then
    echo "Adding source command to bashrc..."; 
    echo "source /opt/ros/fuerte/setup.bash" >> ~/.bashrc;
fi

source /opt/ros/fuerte/setup.bash
apt-get install -y --no-remove python-rosinstall python-rosdep
cd /opt/ros/fuerte/stacks/