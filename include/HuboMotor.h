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
#include "Names.h"
#include "hubo.h"

#define MAX_ANGULAR_VELOCITY 2

using std::string;

class HuboMotor {

private:

    typedef ReferenceChannel::Mode Mode;

    //Identification
    string name;

    //Output Data
    double currGoal;                //Goal position in radians
    double interStep;                //Current interpolated step in radians
    double frequency;                //Interpolation Frequency
    double interVel;                //Current interpolated velocity in rad/sec
    hubo_mode_type_t mode;          // hubo-ach interpretation mode

    //Sensor Data
    double currVel;                    //Reported velocity (units?)
    double currPos;                    //Reported position in radians
    double currCurrent;                //Reported current (units?)
    double currTemp;                //Reported temperature (units?)

    //Internal State
    bool enabled;                    //Whether the motor has motion enabled
    bool homed;                        //Whether the motor has been homed or not
    bool zeroed;                    //Whether the sensors have been zeroed or not
    int errors;                        //Collection of error flags

public:

    HuboMotor();
    HuboMotor(const HuboMotor& rhs);

    void setName(string &name);
    void setGoalPosition(double rads);
    void setInterStep(double rads);
    void setInterVelocity(double omega);
    void setFrequency(double frequency);
    void update(double position, double velocity, double temperature, double current, bool homed, int errors);
    void setEnabled(bool enabled);
    void setHomed(bool homed);
    void setZeroed(bool zeroed);
    void setMode(Mode mode);
    double interpolate();

    string& getName();
    double getGoalPosition();
    double getPosition();
    double getInterStep();
    double getVelocity();
    double getTemperature();
    double getCurrent();
    bool isEnabled();
    bool isHomed();
    bool isZeroed();
    bool hasError();
    bool hasError(PROPERTY error);
    bool requiresMotion();
    Mode getMode();
};

#endif
