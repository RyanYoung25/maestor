#!/bin/bash
#
# This script installs ROS Fuerte with the Orocos Toolchain and
# openRAVE stacks.
#
# Options:
# 	-y		: Quiet install. Answers yes to any prompts.
#	--auto-yes	: See -y
#
# Dependencies:
# 	<Maestro install dir>/maestro/utils.sh
#	/etc/apt/sources.list    (If you're missing this, reinstall Ubuntu.)	
#	
# Blacklist:
#	/opt/ros/fuerte/stacks/armnavigation	(Mercury complains)
#	/opt/ros/fuerte/stacks/maestro		(No previous installs)
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

echo "ROS-Fuerte Maestor installation Script"
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

#ROS
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

SKIP_OROCOS=false
#OROCOS
if [[ -d orocos ]]; then
	if [[ "$QUIET" == false ]]; then
		echo ""
		echo "There seems to be a previous installation of Orocos. Would you like to remove it?"
		select yn in "Yes" "No"; do
			case $yn in
			Yes ) 
				echo "Removing previous installation." 
				rm -rf orocos
				break;;
			No ) 
				echo "Orocos install will be untouched."
				sleep 2
				SKIP_OROCOS=true
				break;;
			esac
		done
		echo ""
	elif [[ "$QUIET" == true ]]; then
		echo "A previous Orocos installation has been detected. Orocos install
will be skipped."
		SKIP_OROCOS=true
		sleep 2
	fi
fi

if [[ $SKIP_OROCOS == false ]]; then
	mkdir orocos
	cd /opt/ros/fuerte/stacks/orocos
	apt-get install -y --no-remove libreadline-dev omniorb omniidl omniorb-nameserver libomniorb4-1 libomniorb4-dev libomnithread3-dev libomnithread3c2 gccxml antlr libantlr-dev libxslt1-dev liblua5.1-0-dev ruby1.8-dev libruby1.8 rubygems1.8 
	git clone --recursive git://gitorious.org/orocos-toolchain/orocos_toolchain.git
	git clone http://git.mech.kuleuven.be/robotics/rtt_ros_integration.git
	git clone http://git.mech.kuleuven.be/robotics/rtt_ros_comm.git
	git clone http://git.mech.kuleuven.be/robotics/rtt_common_msgs.git
	git clone http://git.mech.kuleuven.be/robotics/rtt_geometry.git
	roscd orocos_toolchain
	git checkout toolchain-2.5
	git submodule init
	git submodule update
	git submodule foreach git checkout toolchain-2.5
	source env.sh
	apt-get install -y --no-remove libboost-dev
	rosmake orocos_toolchain rtt_ros_integration rtt_ros_comm rtt_common_msgs rtt_geometry
fi
#OPENRAVE
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
if [ $# -lt 1 ]
then
	echo ""
	echo ""
	echo "Would you like a link to your Maestro install in /opt/ros/fuerte/stacks?"
	select yn in "Yes" "No"; do
        	case $yn in
                	Yes ) echo "Creating symbolic link in /opt/ros/fuerte/stacks..."; ln -s $installDir/../ maestro; break;;
                	No ) echo "Skipping symbolic link creation..."; break;;
        	esac
	done
	echo ""
elif [[ "$1" == "-y" ||  "$1" == "--auto-yes" ]]
then
	echo "Creating symbolic link in /opt/ros/fuerte/stacks..."; 
	ln -s $installDir/../ maestro;
fi
rosmake maestro
echo "Installation complete."
