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

#define CONFIG_PATH "/opt/ros/fuerte/stacks/maestor/config/config.txt"
#define LOG_PATH "/opt/ros/fuerte/stacks/maestor/logs/"
#define HARDWARE true
#define SIMULATION false

#include "ros/ros.h"

#include <vector>
#include <queue>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <sys/time.h>
#include <string>
#include <stdio.h>

#include "Scheduler.h"
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
#include "BalanceController.h"

using ros::NodeHandle;
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

class RobotControl {

private:
    typedef HuboState::Components Components;
    typedef HuboState::Motors Motors;
    typedef Trajectory::Header Header;
    typedef Names::Properties Properties;
    typedef Names::Commands Commands;

public:
    RobotControl();
    ~RobotControl();

    void updateHook();
    void initRobot(string path);
    void setPeriod(double period);

    //JOINT MOVEMENT API
    void set(string name, string property, double value);
    void setProperties(string names, string properties, string values);

    // Control Commands
    void debugControl(int board, int operation);
    void setDelay(int us);
    void command(string name, string target);
    //void handleMessage(MaestroCommand message);

    // Feedback Commands
    bool requiresMotion(string name);
    double get(string name, string property);
    string getProperties(string name, string properties);
    void updateState();

    // Feedback for walking
    //double getZMP(int typeValue);
    void DSPControl();
    void vibrationControl();
    double DampingControl();
    void ZMPInitialization();


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
    void setSimType();
    bool setAlias(string name, string alias);
    vector<string> splitFields(string input);
    string getDefaultInitPath(string path);


private:

    
    HuboState *state;
    PowerControlBoard *power;
    BalanceController *balancer;

    //map< string, vector<float> > gestures;
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
    double PERIOD;
    bool trajStarted, terminateTraj;
    bool printNow, enableControl;
    int delay;
    bool interpolation, override;
    bool RUN_TYPE;
    bool balanceOn;

    //ros::NodeHandle nh;
	bool maestro_run_type;
};

#endif
