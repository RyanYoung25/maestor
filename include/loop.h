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

using ros::NodeHandle;
using ros::ServiceServer;
using ros::init;

void setRealtime();

using std::cout;
using std::endl;

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