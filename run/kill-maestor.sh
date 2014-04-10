#! /bin/bash

MaestorPID=$(pgrep maestor)
RoscorePID=$(pgrep roscore)
SimulationPID=$(pgrep hubo-ach)

sudo kill -2 $MaestorPID
kill -2 $RoscorePID
kill -9 $SimulationPID
hubo-ach killall


