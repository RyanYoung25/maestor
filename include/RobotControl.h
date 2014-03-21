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
#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#define CONFIG_PATH "/opt/ros/fuerte/stacks/maestro/maestro/config/config.txt"
#define LOG_PATH "/opt/ros/fuerte/stacks/maestro/maestro/logs/"
#define HARDWARE true
#define SIMULATION false

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/scripting/ProgramInterface.hpp>
#include <rtt/scripting/ScriptingService.hpp>
#include <hubomsg/typekit/HuboState.h>
#include <hubomsg/typekit/HuboJointState.h>
#include <hubomsg/typekit/HuboCommand.h>
#include <hubomsg/typekit/HuboJointCommand.h>
#include <hubomsg/typekit/AchCommand.h>
#include <hubomsg/typekit/HuboIMU.h>
#include <hubomsg/typekit/HuboFT.h>
#include <hubomsg/typekit/MaestroCommand.h>
#include <hubomsg/typekit/MaestroMessage.h>
#include <vector>
#include <queue>
#include <map>
#include "CommHandler.h"
#include "HuboState.h"
#include "HuboMotor.h"
#include "MotorBoard.h"
#include "IMUBoard.h"
#include "FTSensorBoard.h"
#include "PowerControlBoard.h"
#include "Names.h"
#include "Singleton.h"
#include "CommandChannel.h"
#include "ReferenceChannel.h"
#include "StateChannel.h"
#include "SimChannels.h"
#include "Trajectory.h"
#include "TrajHandler.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <sys/time.h>
#include <string>
#include <stdio.h>

using std::queue;
using std::vector;
using std::map;
using std::string;
using std::ofstream;
using std::ifstream;
using std::ostringstream;
using std::istringstream;
using std::cout;
using std::endl;
using hubomsg::MaestroCommand;
using hubomsg::MaestroMessage;
using hubomsg::AchCommand;

using RTT::TaskContext;

class RobotControl : public TaskContext{

private:
    typedef HuboState::Motors Motors;
    typedef HuboState::Boards Boards;
    typedef HuboState::Properties Properties;
    typedef HuboState::FTSensors FTSensors;
    typedef HuboState::IMUSensors IMUSensors;
    typedef Trajectory::Header Header;

public:
    RobotControl(const string&);
    ~RobotControl();

    void updateHook();
    void initRobot(string path);

    //JOINT MOVEMENT API
    void set(string name, string property, double value);
    void setProperties(string names, string properties, string values);

    // Control Commands
    void debugControl(int board, int operation);
    void setDelay(int us);
    void runGesture(string name);
    void command(string name, string target);
    void handleMessage(MaestroCommand message);

    // Feedback Commands
    bool requiresMotion(string name);
    double get(string name, string property);
    string getProperties(string name, string properties);
    void updateState();

    // Parameter Commands
    void setMode(string mode, bool value);

    // Trajectory Commands
    bool loadTrajectory(string name, string path, bool read);
    bool ignoreFrom(string name, string col);
    bool ignoreAllFrom(string name);
    bool unignoreFrom(string name, string col);
    bool unignoreAllFrom(string name);
    bool setTrigger(string name, int frame, string target);
    bool extendTrajectory(string name, string path);
    void startTrajectory(string name);
    void stopTrajectory(string name);

    // Configuration Commands
    bool getRunType();
    bool setAlias(string name, string alias);
    vector<string> getGestureScripts(string path);
    vector<string> splitFields(string input);
    string getDefaultInitPath(string path);


private:

    //SUBSCRIBE
    InputPort<MaestroCommand> *commandPort;
    CommHandler *commHandler;

    //PUBLISH
    OutputPort<MaestroMessage> *messageDownPort;

    HuboState *state;
    PowerControlBoard *power;

    map< string, vector<float> > gestures;
    map< string, COMMAND > commands;
    ofstream tempOutput;
    ifstream trajInput;
    TrajHandler trajectories;

    CommandChannel *commandChannel;
    ReferenceChannel *referenceChannel;
    StateChannel *stateChannel;
    SimChannels *simChannels;

    int written;
    int frames;
    bool trajStarted, terminateTraj;
    bool printNow, enableControl;
    int delay;
    bool interpolation, override;
    bool RUN_TYPE;

	ros::NodeHandle nh;
	bool maestro_run_type;
};

#endif
