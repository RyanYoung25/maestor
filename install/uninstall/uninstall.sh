#!/bin/bash
echo "ROS-Maestor Uninstall Script"
echo "Version 1.0"
echo ""

echo "Would you like to uninstall ROS Fuerte?"
select yn in "Yes" "No"; do
	case $yn in
		Yes ) echo "Removing ROS Fuerte..."; sudo apt-get purge ros-fuerte-desktop-full; sudo apt-get autoremove; break;;
		No ) echo "Skipping removal of ROS Fuerte..."; break;;
	esac
done

echo ""
echo ""


echo "Would you like to clean your hubo-ach environment?"
select yn in "Yes" "No"; do
        case $yn in
                Yes ) echo "Running Hubo-Ach clean script..."; bash hubo-ach-clean-all.sh; sudo apt-get purge libach1 libach-dev ach-utils hubo-ach hubo-ach-dev;  break;;
                No ) echo "Skipping cleaning of Hubo-Ach..."; break;;
        esac
done

echo ""
echo ""

echo "Would you like to remove your OpenHUBO Install?"
select yn in "Yes" "No"; do
        case $yn in
                Yes ) echo "Removing OpenHUBO Install Dir..."; sudo rm -rf $OPENHUBO_DIR;  break;;
                No ) echo "Skipping cleaning of Hubo-Ach..."; break;;
        esac
done

echo ""
echo ""

echo "Uninstall complete. All requested changes have been implemented."
echo "There may be some lingering packages in your ROS Stacks folder."
echo "Would you like to delete them?"

select yn in "Yes" "No"; do
        case $yn in
                Yes ) echo "Removing ROS folder..."; sudo rm -rf /opt/ros/fuerte/stacks;  break;;
                No ) echo "Skipping removal of ROS folder..."; break;;
        esac
done

echo "Finished. Exiting..."
 
