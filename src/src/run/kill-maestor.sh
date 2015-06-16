#! /bin/bash

RoscorePID=$(pgrep roscore)
SimulationPID=$(pgrep hubo-ach)


kill -2 $RoscorePID
kill -9 $SimulationPID >> /dev/null 2>/dev/null
hubo-ach killall >> /dev/null 2>/dev/null
sudo pkill -9 maestor
echo "Maestor shut down successfully"


