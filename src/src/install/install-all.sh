#!/bin/bash
#
# Comprehensive install script for Maestor, and all dependencies.
# Installs Ros Fuerte, OpenRAVE, Maestor, Hubo-Ach, OpenHubo.
#
# Options: None
# Dependencies:
#	<Maestor Install Dir>/utils.sh
#	install-ros-fuerte.sh
#	install-hubo-ach.sh
#   install-openrave.sh
#	install-openHubo.sh
#
#

# Change directory to the script's directory
if [[ `echo "$0" | grep "/" | wc -l` > 0 ]]; then
    cd ${0%/*}
fi

source ../utils.sh

#set -e
echo "Comprehensive Install Script for Maestor"
echo "Version $VERSION"
echo ""

DEPENDENCY_DIRS=""
DEPENDENCY_FILES="install-maestor.sh install-ros-fuerte.sh install-hubo-ach.sh install-openHubo.sh install-openrave.sh"
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

echo "Dependencies satisfied."
if [[ $? != 0 ]]; then 
    exit $?; 
fi
echo "Installing ros-fuerte..."
sudo bash install-ros-fuerte.sh -y
if [[ $? != 0 ]]; then 
    echo "ros-fuerte did not install. Try running install-ros-fuerte.sh"
    exit $?
fi
echo "Installing openrave..."
sudo bash install-openrave.sh -y
if [[ $? != 0 ]]; then 
    echo "openrave did not install. Try running install-openrave.sh"
    exit $?
fi
echo "Installing Hubo-ACH..."
sudo bash install-hubo-ach.sh -y
if [[ $? != 0 ]]; then 
    echo "hubo-ach did not install. Try running install-hubo-ach.sh"
    exit $? 
fi
echo "Installing MAESTOR..."
bash install-maestor.sh -y
if [[ $? != 0 ]]; then 
    echo "maestor did not install. Try running install-maestor.sh"
    exit $?
fi
echo "Updating PYTHONPATH..."
bash update-pythonpath.sh -y
if [[ $? != 0 ]]; then 
    echo "The PYTHONPATH was not updated. Try running update-pythonpath.sh"
    exit $?
fi
echo "Installing OpenHUBO..."
bash install-openHubo.sh -y
if [[ $? != 0 ]]; then 
    echo "openHubo did not install. Try running install-openHubo.sh"
    exit $? 
fi
echo "Install Complete. Exiting..."
