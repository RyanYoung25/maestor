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
#ifndef HUBOMOTOR_H
#define HUBOMOTOR_H

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>

#include "ReferenceChannel.h"
#include "RobotComponent.h"
#include "Interpolable.h"
#include "Names.h"
#include "hubo.h"

#define MAX_ANGULAR_VELOCITY 2

using std::string;

class HuboMotor : public RobotComponent, public Interpolable {

private:

    typedef ReferenceChannel::Mode Mode;

    //Identification
    Mode mode;                      //hubo-ach interpretation mode

    //Internal State
    bool enabled;                    //Whether the motor has motion enabled
    int boardNum;                   //Board number as referenced in Hubo-ach

    //Soft limits for the motors
    double lowerLim;
    double upperLim;


public:

    HuboMotor();
    virtual ~HuboMotor();

    bool get(PROPERTY property, double& value);
    bool set(PROPERTY property, double value);

    void setBoardNum(int boardNum);
    void setLowerLim(double value);
    void setUpperLim(double value);

    bool requiresMotion();   
};

#endif
