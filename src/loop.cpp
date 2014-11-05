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

#include "loop.h"

RobotControl robot;
/**
 * The main method that starts MAESTOR! This is where the ros node is made and all of the 
 * services are advertised. There are also wrappers in here for all of the service methods that 
 * link up to the ones that are in RobotControl. 
 * @param  argc Number of arguments
 * @param  argv Argument array
 * @return      0 when it shuts down
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "Maestor"); 
    setRealtime();

    //Check for the run type

    if(argc == 2)
    {
        if(strcmp(argv[1], "sim") == 0)
        {
            //If simulation set the runtime to sim
            robot.setSimType();
        }
    }
    //Init the node
    NodeHandle n; //Fully initializes the node
    Scheduler timer(FREQ_200HZ);
    robot.setPeriod(1.0/timer.getFrequency());
    ServiceServer srv = n.advertiseService("fib", &fib);
    ServiceServer Initsrv = n.advertiseService("initRobot", &initRobot);
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

    ServiceServer SetPropsrv = n.advertiseService("setProperty", &setProperty);

    while (ros::ok()) {
        ros::spinOnce();
        robot.updateHook();
        timer.sleep();
        timer.update();
    }
    return 0;
}

/**
 * Give us a real time priority 
 */
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

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool initRobot(maestor::initRobot::Request &req, maestor::initRobot::Response &res)
{
    robot.initRobot(req.path);
    return true;

}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool setProperties(maestor::setProperties::Request &req, maestor::setProperties::Response &res)
{
    robot.setProperties(req.names, req.properties, req.values);
    return true;
}

// Control Commands

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool command(maestor::command::Request &req, maestor::command::Response &res)
{
    robot.command(req.name, req.target);
    return true;
}

// Feedback Commands

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool requiresMotion(maestor::requiresMotion::Request &req, maestor::requiresMotion::Response &res)
{
    res.requiresMotion = robot.requiresMotion(req.name);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool getProperties(maestor::getProperties::Request &req, maestor::getProperties::Response &res)
{
    res.properties = robot.getProperties(req.name, req.properties);
    return true;
}

// Trajectory Commands

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool loadTrajectory(maestor::loadTrajectory::Request &req, maestor::loadTrajectory::Response &res)
{
    res.success = robot.loadTrajectory(req.name, req.path, req.read);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool ignoreFrom(maestor::ignoreFrom::Request &req, maestor::ignoreFrom::Response &res)
{
    res.success = robot.ignoreFrom(req.name, req.col);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool ignoreAllFrom(maestor::ignoreAllFrom::Request &req, maestor::ignoreAllFrom::Response &res)
{
    res.success = robot.ignoreAllFrom(req.name);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool unignoreFrom(maestor::unignoreFrom::Request &req, maestor::unignoreFrom::Response &res)
{
    res.success = robot.unignoreFrom(req.name, req.col);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool unignoreAllFrom(maestor::unignoreAllFrom::Request &req, maestor::unignoreAllFrom::Response &res)
{
    res.success = robot.unignoreAllFrom(req.name);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool setTrigger(maestor::setTrigger::Request &req, maestor::setTrigger::Response &res)
{
    res.success = robot.setTrigger(req.name, req.frame, req.target);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool extendTrajectory(maestor::extendTrajectory::Request &req, maestor::extendTrajectory::Response &res)
{
    res.success = robot.extendTrajectory(req.name, req.path);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool startTrajectory(maestor::startTrajectory::Request &req, maestor::startTrajectory::Response &res)
{
    robot.startTrajectory(req.name);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool stopTrajectory(maestor::stopTrajectory::Request &req, maestor::stopTrajectory::Response &res)
{
    robot.stopTrajectory(req.name);
    return true;
}

/**
 * Wrapper
 * @param  req The ROS request service part
 * @param  res The ROS response service part
 * @return     True
 */
bool setProperty(maestor::setProperty::Request &req, maestor::setProperty::Response &res)
{
    robot.set(req.name, req.property, req.value);
    return true;
}
