#!/bin/bash
#
# Comprehensive install script for Maestro, and all dependencies.
# Installs Ros Fuerte, Orocos, OpenRAVE, Maestro, Hubo-Ach, OpenHubo.
#
# Options: 
#	-y	Quiet install. Will not prompt for install dir.
#
# Dependencies:
#	<Maestro Install Dir>/maestro/utils.sh
#
# Author: Solis Knight
# Date: July 2013
#

# Change directory to the script's directory
cd ${0%/*}

source ../utils.sh

set -e
echo "OpenHUBO Maestor installation Script"
echo "Version $VERSION"
echo ""

if [[ $# > 0 && $1 == "-y" ]]; then
	QUIET=true
fi

if [[ -z "$QUIET" ]]; then
	read -p "Please enter installation directory (No trailing '/' Please): " installDir
else
	installDir=~/
fi

if [[ ! -d "$installDir" ]]; then
	mkdir $installDir
	if [[ ! -d "$installDir" ]]; then
		echo "Failed to create install directory."
		exit 1
	fi
fi

cd "$installDir"

git clone https://github.com/daslrobotics/openHubo

cd openHubo

git checkout release/0.8.1

source /opt/ros/fuerte/stacks/openrave_planning/openrave/openrave.bash

sh -c './setup'

# if [[ -z $(grep "source $installDir/openHubo/env.sh" ~/.bashrc)  ]]; then
# 	if [[ -z "$QUIET" ]]; then
# 		echo ""
# 		echo "Would you like to add a source line to your bashrc file?"
# 		select yn in "Yes" "No"; do
#         		case $yn in
#                 	Yes )
# 				echo "Adding source command to bashrc..."; 
# 				echo "source $installDir/env.sh" >> ~/.bashrc; 
# 				break;;
#                 	No ) 
# 				echo "Skipping modification of bashrc..."; 
# 				break;;
#         		esac
# 		done
# 	else
# 		echo "source $installDir/env.sh" >> ~/.bashrc;
# 	fi
# fi
source $installDir/openHubo/env.sh

# Ach Python Bindings
sudo apt-get install python-pip
sudo pip install http://code.golems.org/src/ach/py_ach-latest.tar.gz

