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

#include "MotorBoard.h"
#include "pugixml.hpp"
#include "Names.h"
#include <map>
#include <string>
#include <queue>
#include <iostream>

#include "RobotComponent.h"
#include "MetaJointController.h"

#include "HuboMotor.h"
#include "FTSensorBoard.h"
#include "IMUBoard.h"
#include "MetaJoint.h"
#include "ArmMetaJoint.h"
#include "NeckRollPitch.h"
#include "ArmWristXYZ.h"
#include "LowerBodyLeg.h"

using std::map;
using std::string;
using std::cout;
using std::endl;

using pugi::xml_document;
using pugi::xml_node;
using pugi::xml_object_range;

/*
enum MOTOR_NAME {
    RHY, RHR, RHP, RHP1, RHP2, RKP, RKN1, RKN2, RAP, RAR, LHY, LHR, LHP, LHP1, LHP2,
    LKP, LKN1, LKN2, LAP, LAR, RSP, RSR, RSY, REB, LSP, LSR, LSY, LEB, RWY,
    RWP, LWY, LWP, NKY, NK1, NK2, WST, RH0, RH1, RH2, RH3, RH4, RH5,
    LH0, LH1, LH2, LH3, LH4, LH5
} ;
*/

class HuboState : public Singleton<HuboState> {
    friend class Singleton<HuboState>;
public:
    typedef vector< RobotComponent* > Components;
    typedef vector< HuboMotor* > Motors;

private:
    
    Components components;
    Motors motors;
    vector< MetaJointController* > controllers;
    map< string, RobotComponent* > index;

protected:
    HuboState();
    ~HuboState();

public:

    void initHuboWithDefaults(string path, double frequency);

    bool setAlias(string name, string alias);
    bool nameExists(string name);

    RobotComponent* getComponent(string name);

    const Components &getComponents();
    const Motors &getMotors();

private:

    RobotComponent* HuboMotorFromXML(xml_node node, HuboMotor* component, double frequency);
    RobotComponent* FTSensorFromXML(xml_node node, FTSensorBoard* component);
    RobotComponent* IMUSensorFromXML(xml_node node, IMUBoard* component);
    RobotComponent* MetaJointFromXML(xml_node node, MetaJoint* component, double frequency);

    bool addComponentFromXML(xml_node node, RobotComponent* component, bool back);
    bool addMetaJointControllerFromXML(xml_node node, MetaJointController* controller, string type, double frequency);

    void reset();
};
#endif
