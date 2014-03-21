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
#include "HuboMotor.h"

HuboMotor::HuboMotor(){

    currGoal = 0;
    interStep = 0;
    interVel = .3; //Default 1/3 of a radian per second.
    mode = HUBO_REF_MODE_REF_FILTER;

    currVel = 0;
    currPos = 0;
    currCurrent = 0;
    currTemp = 0;
    frequency = 0;

    enabled = false;
    homed = false;
    zeroed = false;
}

HuboMotor::HuboMotor(const HuboMotor& rhs){

    //NEW_DATA
    this->name = rhs.name;
    this->currGoal = rhs.currGoal;
    this->interStep = rhs.interStep;
    this->interVel = rhs.interVel;
    this->mode = rhs.mode;

    this->currVel = rhs.currVel;
    this->currPos = rhs.currPos;
    this->currCurrent = rhs.currCurrent;
    this->currTemp = rhs.currTemp;

    this->enabled = rhs.enabled;
    this->homed = rhs.homed;
    this->zeroed = rhs.zeroed;
}

bool HuboMotor::requiresMotion(){
    return currGoal != interStep;
}

void HuboMotor::setMode(Mode mode){
    this->mode = mode;
}

double HuboMotor::interpolate(){
    if (frequency == 0) return interStep; //If the frequency is 0, no motion occurs.

    const float LEAP_PERCENTAGE = .5;
    const double MIN_STEP = .00001;
    const double MAX_STEP = interVel/frequency; //Radians per second, divided by our operating frequency.

    double error = currGoal - interStep;
    if (error == 0)
        return currGoal;
    double output = currGoal;

    if((fabs(error) > MIN_STEP)){
        output = (LEAP_PERCENTAGE * error);

        if(fabs(output) > MAX_STEP)
            output = output < 0 ? -MAX_STEP : MAX_STEP;

    } else
        output = error;

    output += interStep;
    interStep = output;
    return interStep;
}

void HuboMotor::setName(string &name){
    this->name = name;
}

void HuboMotor::setGoalPosition(double rads){
    currGoal = rads;
}

void HuboMotor::setInterVelocity(double omega){
    interVel = omega;
}

void HuboMotor::setFrequency(double frequency){
    this->frequency = frequency;
}

void HuboMotor::update(double position, double velocity, double temperature, double current, bool homed, int errors){
    currPos = position;
    currVel = velocity;
    currTemp = temperature;
    currCurrent = current;
    this->homed = homed;
    this->errors = errors;
}

void HuboMotor::setEnabled(bool enabled){
    this->enabled = enabled;
    //interStep = currGoal; //If we have recently changed from non-interpolation to interpolation, the step MUST be updated.
}

void HuboMotor::setInterStep(double rads){
    //This method should ONLY be used when switching control mode to interpolation.
    //The argument to this method should be the current actual position of the motor.
    this->interStep = rads;
}

void HuboMotor::setZeroed(bool zeroed){
    this->zeroed = zeroed;
}

string& HuboMotor::getName(){
    return name;
}

double HuboMotor::getGoalPosition(){
    return currGoal;
}

double HuboMotor::getPosition(){
    return currPos;
}

double HuboMotor::getInterStep(){
    return interStep;
}

double HuboMotor::getVelocity(){
    return currVel;
}

double HuboMotor::getTemperature(){
    return currTemp;
}

double HuboMotor::getCurrent(){
    return currCurrent;
}

bool HuboMotor::isEnabled(){
    return enabled;
}

bool HuboMotor::isHomed(){
    return homed;
}

bool HuboMotor::isZeroed(){
    return zeroed;
}

bool HuboMotor::hasError(){
    return errors != 0;
}

bool HuboMotor::hasError(PROPERTY error){
    switch (error){
    case JAM_ERROR:
        return (bool)(errors & 0x200);  // 1000000000
    case PWM_SATURATED_ERROR:
        return (bool)(errors & 0x100);  // 0100000000
    case BIG_ERROR:
        return (bool)(errors & 0x80);   // 0010000000
    case ENC_ERROR:
        return (bool)(errors & 0x40);   // 0001000000
    case DRIVE_FAULT_ERROR:
        return (bool)(errors & 0x20);   // 0000100000
    case POS_MIN_ERROR:
        return (bool)(errors & 0x10);   // 0000010000
    case POS_MAX_ERROR:
        return (bool)(errors & 0x8);    // 0000001000
    case VELOCITY_ERROR:
        return (bool)(errors & 0x4);    // 0000000100
    case ACCELERATION_ERROR:
        return (bool)(errors & 0x2);    // 0000000010
    case TEMP_ERROR:
        return (bool)(errors & 0x1);    // 0000000001
    default:
        return false;
    }
}

HuboMotor::Mode HuboMotor::getMode(){
    return mode;
}
