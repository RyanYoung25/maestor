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
#ifndef HUBOSTATE_H
#define HUBOSTATE_H

#include "HuboMotor.h"
#include "MotorBoard.h"
#include "FTSensorBoard.h"
#include "pugixml.hpp"
#include "IMUBoard.h"
#include "Names.h"
#include <rtt/Port.hpp>
#include <map>
#include <string>
#include <queue>
#include <iostream>


using namespace RTT;
using std::map;
using std::string;
using std::queue;
using std::cout;
using std::endl;

using pugi::xml_document;
using pugi::xml_node;

/*
enum MOTOR_NAME {
    RHY, RHR, RHP, RHP1, RHP2, RKP, RKN1, RKN2, RAP, RAR, LHY, LHR, LHP, LHP1, LHP2,
    LKP, LKN1, LKN2, LAP, LAR, RSP, RSR, RSY, REB, LSP, LSR, LSY, LEB, RWY,
    RWP, LWY, LWP, NKY, NK1, NK2, WST, RH0, RH1, RH2, RH3, RH4, RH5,
    LH0, LH1, LH2, LH3, LH4, LH5
} ;
*/

class HuboState {
public:
    typedef vector< MotorBoard* > Boards;
    typedef map< string, HuboMotor* > Motors;
    typedef map< string, PROPERTY > Properties;
    typedef map< string, FTSensorBoard* > FTSensors;
    typedef map< string, IMUBoard* > IMUSensors;

private:
    
    Boards boards;
    IMUBoard *IMU0, *IMU1, *IMU2;
    FTSensorBoard *leftWrist, *rightWrist, *leftAnkle, *rightAnkle;

    Motors motorMap;
    FTSensors FTSensorMap;
    IMUSensors IMUSensorMap;
    Properties propertyMap;

public:

    HuboState();

    HuboState(const HuboState& rhs);
    void initHuboWithDefaults(string path, double frequency);

    HuboMotor* getMotorByName(string name);

    void addBoard(MotorBoard* board);

    Boards &getBoards();
    Motors &getMotorMap();
    Properties &getPropertyMap();
    FTSensors &getFTSensorMap();
    IMUSensors &getIMUSensorMap();
};
#endif
