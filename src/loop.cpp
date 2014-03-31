/*
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

using ros::NodeHandle;
using ros::ServiceServer;
using ros::init;

void setRealtime();

using std::cout;
using std::endl;
*/
#include "loop.h"

RobotControl robot;

int main(int argc, char **argv) {
    ros::init(argc, argv, "Maestor"); 
    setRealtime();
    //Init the node
    NodeHandle n; //Fully initializes the node
    Scheduler timer(FREQ_200HZ);
    ServiceServer srv = n.advertiseService("fib", &fib);
    ServiceServer SPsrv = n.advertiseService("setProperties", &setProperties);
    ServiceServer Comsrv = n.advertiseService("command", &command);
    ServiceServer RMsrv = n.advertiseService("requiresMotion", &requiresMotion);
    ServiceServer GPsrv = n.advertiseService("getProperties", &getProperties);
    ServiceServer LTsrv = n.advertiseService("loadTrajectory", &loadTrajectory);
    ServiceServer IFsrv = n.advertiseService("ignoreFrom", &ignoreFrom);
    ServiceServer IAFsrv = n.advertiseService("ignoreAllFrom", &ignoreAllFrom);
    ServiceServer UFsrv = n.advertiseService("unignoreFrom", &unignoreFrom);
    ServiceServer UAFsrv = n.advertiseService("unignoreAllFrom", &unignoreAllFrom);
    ServiceServer STsrv = n.advertiseService("setTrigger", &setTrigger);
    ServiceServer ETsrv = n.advertiseService("extendTrajectory", &extendTrajectory);
    ServiceServer StTsrv = n.advertiseService("startTrajectory", &startTrajectory);
    ServiceServer SpTsrv = n.advertiseService("stopTrajectory", &stopTrajectory);

    while (ros::ok()) {
        ros::spin();

        robot.updateHook();

        timer.sleep();
        timer.update();
    }
    return 0;
}

void setRealtime(){
    // Taken from hubo-ach/src/hubo-daemonizer.c

    struct sched_param param;

    // Declare ourself as a real time task
    param.sched_priority = 49;
    if(sched_setscheduler(getpid(), SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
        exit(-1);
    }
}

bool setProperties(maestor::setProperties::Request &req, maestor::setProperties::Response &res)
{
    robot.setProperties(req.names, req.properties, req.values);
    return true;
}

// Control Commands
bool command(maestor::command::Request &req, maestor::command::Response &res)
{
    robot.command(req.name, req.target);
    return true;
}
//bool handleMessage(MaestroCommand message);

// Feedback Commands
bool requiresMotion(maestor::requiresMotion::Request &req, maestor::requiresMotion::Response &res)
{
    res.requiresMotion = robot.requiresMotion(req.name);
    return true;
}

bool getProperties(maestor::getProperties::Request &req, maestor::getProperties::Response &res)
{
    res.properties = robot.getProperties(req.name, req.properties);
    return true;
}

// Trajectory Commands
bool loadTrajectory(maestor::loadTrajectory::Request &req, maestor::loadTrajectory::Response &res)
{
    res.success = robot.loadTrajectory(req.name, req.path, req.read);
    return true;
}

bool ignoreFrom(maestor::ignoreFrom::Request &req, maestor::ignoreFrom::Response &res)
{
    res.success = robot.ignoreFrom(req.name, req.col);
    return true;
}

bool ignoreAllFrom(maestor::ignoreAllFrom::Request &req, maestor::ignoreAllFrom::Response &res)
{
    res.success = robot.ignoreAllFrom(req.name);
    return true;
}

bool unignoreFrom(maestor::unignoreFrom::Request &req, maestor::unignoreFrom::Response &res)
{
    res.success = robot.unignoreFrom(req.name, req.col);
    return true;
}

bool unignoreAllFrom(maestor::unignoreAllFrom::Request &req, maestor::unignoreAllFrom::Response &res)
{
    res.success = robot.unignoreAllFrom(req.name);
    return true;
}

bool setTrigger(maestor::setTrigger::Request &req, maestor::setTrigger::Response &res)
{
    res.success = robot.setTrigger(req.name, req.frame, req.target);
    return true;
}

bool extendTrajectory(maestor::extendTrajectory::Request &req, maestor::extendTrajectory::Response &res)
{
    res.success = robot.extendTrajectory(req.name, req.path);
    return true;
}

bool startTrajectory(maestor::startTrajectory::Request &req, maestor::startTrajectory::Response &res)
{
    robot.startTrajectory(req.name);
    return true;
}

bool stopTrajectory(maestor::stopTrajectory::Request &req, maestor::stopTrajectory::Response &res)
{
    robot.stopTrajectory(req.name);
    return true;
}