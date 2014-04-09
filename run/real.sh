#!/bin/bash
if [ $(pgrep hubo-daemon) ]; then
    echo "Hubo-Ach daemon is already running. Terminating..."
    hubo-ach killall &> /dev/null
    echo "Hubo-Ach daemon terminated."
fi


if [ $(pgrep roscore) ]; then
    echo "roscore is already running."
else
    roscore &
fi

sudo openvt -- hubo-ach start 

while [ ! $(pgrep hubo-daemon) ]; do
	sleep 1
done

while [ ! $(pgrep roscore) ]; do
	sleep 1
done

sudo -E /opt/ros/fuerte/stacks/maestor/bin/maestor


