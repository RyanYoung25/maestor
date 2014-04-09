#!/bin/bash
if [ $(pgrep hubo-daemon) ]; then
    echo "Hubo-Ach daemon is already running. Terminating..."
    hubo-ach killall &> /dev/null
    echo "Hubo-Ach daemon terminated."
fi

openvt -- roscore 

sudo openvt -- hubo-ach start 

while [ ! $(pgrep hubo-daemon) ]; do
	sleep 1
done

while [ ! $(pgrep roscore) ]; do
	sleep 1
done

sudo -E ~/git/git/maestor/bin/maestor


