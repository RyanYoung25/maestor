#!/bin/bash
if [ $(pgrep maestor | wc -w) -gt 1 ]; then
    echo "Maestor is already running"
    echo "If you would like to kill maestor run the command: "
    echo "maestor kill"
    exit 0
fi

if [ $(pgrep hubo-daemon) ]; then
    echo "Hubo-Ach daemon is already running. Terminating..."
    hubo-ach killall &> /dev/null
    echo "Hubo-Ach daemon terminated."
fi

xterm -e "roscore" &

export PYTHONPATH="$PYTHONPATH:/usr/lib/python2.7/dist-packages" 
xterm -e "hubo-ach sim openhubo nophysics simtime" &

while [ ! $(pgrep hubo-daemon) ]; do
    sleep 1
done

sudo -E /opt/ros/fuerte/stacks/maestor/bin/maestor sim &
sleep 1
while [ ! $(pgrep maestor | wc -w) == 2 ]; do
    sleep 1
done
