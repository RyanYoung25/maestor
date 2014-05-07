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
    mode = HUBO_REF_MODE_REF;

    enabled = false;
    boardNum = -1;
}

HuboMotor::~HuboMotor(){};

bool HuboMotor::set(PROPERTY property, double value){

    switch (property){
    case POSITION:
    case GOAL:
        if (value == interStep)
            break;

        double currVel;
        get(VELOCITY, currVel);
        /*
        if ((value > interStep && currVel < 0) || (value < interStep && currVel > 0)){
            cout << "Switching Directions for " << this->getName() << "  currVel: " << currVel << "  currStep: " << currStepCount << "  interStep: " << interStep << endl;
        }
        */

        if (!startParams.valid){
            currStepCount = 0;
            totalStepCount = totalTime(interStep, value, currVel/frequency, interVel) * frequency;
            if (totalStepCount > 1 && fabs(value - interStep) < (interVel/frequency)){
                cout << "Anomaly detected! " << totalStepCount << endl;
            }

            if (totalStepCount > 0) {
                startParams = initFourthOrder(interStep, currVel / frequency, (value + interStep)/2, (double)totalStepCount / 2, value, totalStepCount);
                currParams = startParams;
            }

            lastGoal = interStep;
        } else if (currStepCount != currParams.tv) {

            //William finds this to be questionable methodology due to the difference between predicted step and real encoder position
            //I swear, Will, I have a reason for this!
            double newVia = (startParams.ths + interpolateFourthOrder(currParams, currStepCount));

            totalStepCount = currStepCount + (totalTime(newVia, value, currVel/frequency, interVel) * frequency);
            //totalStepCount = (fabs(interStep - startParams.ths) + fabs(startParams.thf - interStep) + fabs(value - startParams.thf)) * frequency / interVel;
            //Having the start velocity be the current velocity really isn't supported mathematically... :/ but it's not blowing up.
            currParams = initFourthOrder( startParams.ths, currVel/frequency, newVia, currStepCount, value, totalStepCount );
        }
    

        currGoal = value;

        break;
    case INTERPOLATION_STEP:
        interStep = value;
        currParams.valid = false;
        startParams.valid = false;
        break;
    case VELOCITY:
        if (value != 0)
            interVel = value;
        break;
    case GOAL_TIME:
        currStepCount = 0;
        totalStepCount = (int) (value * (frequency / 1000) );
        break;
    case MOTION_TYPE:
        switch ((int)value) {
        case HUBO_REF_MODE_REF:
        case HUBO_REF_MODE_REF_FILTER:
        case HUBO_REF_MODE_COMPLIANT:
        case HUBO_REF_MODE_ENC_FILTER:
        case HUBO_REF_MODE_REFX:
            mode = ((hubo_mode_type_t)value);
            break;
        default:
            cout << "Motion type " << value << " not recognized." << endl;
            return false;
        }
        break;
    case ENABLED:
        enabled = (bool)value;
        break;
    }
    return true;
}

bool HuboMotor::get(PROPERTY property, double& value){

    switch (property){
    case GOAL:
        value = currGoal;
        break;
    case INTERPOLATION_STEP:
        value = interpolate();
        break;
    case GOAL_TIME:
        value = (totalStepCount * 1000) / frequency;
        break;
    case MOTION_TYPE:
        value = mode;
        break;
    case ENABLED:
        value = enabled;
        break;
    case POSITION:
    case VELOCITY:
    case TEMPERATURE:
    case CURRENT:
    case HOMED:
    case ERRORED:
    case JAM_ERROR:
    case PWM_SATURATED_ERROR:
    case BIG_ERROR:
    case ENC_ERROR:
    case DRIVE_FAULT_ERROR:
    case POS_MIN_ERROR:
    case POS_MAX_ERROR:
    case VELOCITY_ERROR:
    case ACCELERATION_ERROR:
    case TEMP_ERROR:
        // Chooses whether to use the name of the board or the board number to request the property from the state channel
        // Prints an error if the request fails
        if ( ( boardNum != -1 ? !stateChannel->getMotorProperty(boardNum, property, value) : !stateChannel->getMotorProperty(getName(), property, value) ) ){
            cout << "Error getting " << Names::getName(property) << " from " << getName() << endl;
            return false;
        }
        break;
    default:
        return false;
    }

    return true;
}

bool HuboMotor::requiresMotion(){
    return interStep != currGoal;
    //return fabs(interStep - currGoal) > .00001;
}

void HuboMotor::setBoardNum(int boardNum){
    this->boardNum = boardNum;
}
