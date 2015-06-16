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
source /opt/ros/fuerte/setup.bash
# Stop execution on any significant error.
#set -e

echo "openRAVE installation Script"
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

add-apt-repository -y ppa:openrave/testing
apt-get update
apt-get install -y --no-remove openrave
cd /opt/ros/fuerte/stacks
svn co https://svn.code.sf.net/p/jsk-ros-pkg/code/trunk .
hg clone https://kforge.ros.org/armnavigation/armnavigation
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_controllers/branches/pr2_controllers-1.4/pr2_controllers_msgs
svn co https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_common/trunk/pr2_msgs
cd /opt/ros/fuerte/stacks/openrave_planning/openrave_robot_control/
rosmake
ln -sf /opt/ros/fuerte/stacks/openrave_planning/openrave/bin/openrave /usr/bin/openrave
ln -sf /opt/ros/fuerte/stacks/openrave_planning/openrave/bin/openrave-config /usr/bin/openrave-config
cd /opt/ros/fuerte/stacks
