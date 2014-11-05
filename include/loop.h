/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <math.h>

#include "Scheduler.h"
#include "servTest.h"
#include "RobotControl.h"
#include "maestor/initRobot.h"
#include "maestor/setProperties.h"
#include "maestor/command.h"
#include "maestor/requiresMotion.h"
#include "maestor/getProperties.h"
#include "maestor/loadTrajectory.h"
#include "maestor/ignoreFrom.h"
#include "maestor/ignoreAllFrom.h"
#include "maestor/unignoreFrom.h"
#include "maestor/unignoreAllFrom.h"
#include "maestor/setTrigger.h"
#include "maestor/extendTrajectory.h"
#include "maestor/startTrajectory.h"
#include "maestor/stopTrajectory.h"
#include "maestor/setProperty.h"

using ros::NodeHandle;
using ros::ServiceServer;
using ros::init;

void setRealtime();

using std::cout;
using std::endl;

bool initRobot(maestor::initRobot::Request &req, maestor::initRobot::Response &res);

bool setProperties(maestor::setProperties::Request &req, maestor::setProperties::Response &res);

// Control Commands
bool command(maestor::command::Request &req, maestor::command::Response &res);
//bool handleMessage(MaestroCommand message);

// Feedback Commands
bool requiresMotion(maestor::requiresMotion::Request &req, maestor::requiresMotion::Response &res);
bool getProperties(maestor::getProperties::Request &req, maestor::getProperties::Response &res);

// Trajectory Commands
bool loadTrajectory(maestor::loadTrajectory::Request &req, maestor::loadTrajectory::Response &res);
bool ignoreFrom(maestor::ignoreFrom::Request &req, maestor::ignoreFrom::Response &res);
bool ignoreAllFrom(maestor::ignoreAllFrom::Request &req, maestor::ignoreAllFrom::Response &res);
bool unignoreFrom(maestor::unignoreFrom::Request &req, maestor::unignoreFrom::Response &res);
bool unignoreAllFrom(maestor::unignoreAllFrom::Request &req, maestor::unignoreAllFrom::Response &res);
bool setTrigger(maestor::setTrigger::Request &req, maestor::setTrigger::Response &res);
bool extendTrajectory(maestor::extendTrajectory::Request &req, maestor::extendTrajectory::Response &res);
bool startTrajectory(maestor::startTrajectory::Request &req, maestor::startTrajectory::Response &res);
bool stopTrajectory(maestor::stopTrajectory::Request &req, maestor::stopTrajectory::Response &res);
bool setProperty(maestor::setProperty::Request &req, maestor::setProperty::Response &res);
