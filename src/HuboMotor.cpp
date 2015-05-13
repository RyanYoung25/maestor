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

/**
 * Create a Hubo Motor object. This represents a hubo joint. It is the
 * building block of all the joints of hubo. 
 */
HuboMotor::HuboMotor(){
    mode = HUBO_REF_MODE_REF;

    enabled = false;
    boardNum = -1;

    //Default limits, 
    upperLim = 3.14;
    lowerLim = -3.14;
}

/**
 * Destructor 
 */
HuboMotor::~HuboMotor(){};

/**
 * Set the property of this hubo motor to a specific value
 * @param  property The property to set 
 * @param  value    The value to set the property to 
 * @return          True on success
 */
bool HuboMotor::set(PROPERTY property, double value){

    switch (property){
    case POSITION:
    case GOAL:
        if (value == interStep)
            break;

        //Soft limits for joint position values
        if (value > upperLim)
        {
            value = upperLim;
        }
        else if(value < lowerLim)
        {
            value = lowerLim;
        }

        double currVel;
        get(VELOCITY, currVel);
        if (!startParams.valid){
            currStepCount = 0;
            totalStepCount = totalTime(interStep, value, currVel/frequency, interVel) * frequency;
            if (totalStepCount > 1 && fabs(value - interStep) < (interVel/frequency)){
                if(totalStepCount > 50){
                    cout << "Anomaly detected! " << totalStepCount << endl;
                }
            }

            if (totalStepCount > 0) {
                startParams = initFourthOrder(interStep, currVel / frequency, (value + interStep)/2, (double)totalStepCount / 2, value, totalStepCount);
                currParams = startParams;
            }

            lastGoal = interStep;
        } else if (currStepCount != currParams.tv) {

            double newVia = (startParams.ths + interpolateFourthOrder(currParams, currStepCount));

            totalStepCount = currStepCount + (totalTime(newVia, value, currVel/frequency, interVel) * frequency);
            currParams = initFourthOrder( startParams.ths, currVel/frequency, newVia, currStepCount, value, totalStepCount );
        }

        currGoal = value;

        break;
    case INTERPOLATION_STEP:
        interStep = value;
        currParams.valid = false;
        startParams.valid = false;
        break;
    case SPEED:
    case VELOCITY:
        if (value > 0)
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

/**
 * Get the value of a property for this specific motor. 
 * @param  property The property to get the value of 
 * @param  value    A pointer that will be filled with the value of the property you queried 
 * @return          True on success
 */
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
    case SPEED:
        value = interVel;
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

/**
 * Returns true if the joint requires motion. I don't know if it is used...
 * @return True if the joint is not at it's goal
 */
bool HuboMotor::requiresMotion(){
    return interStep != currGoal;
}

/**
 * Set the board number of this joint
 * @param boardNum The board number to set it to
 */
void HuboMotor::setBoardNum(int boardNum){
    this->boardNum = boardNum;
}

/**
 * Set the upper limit for this joint, the limit is set at 
 * configuration time from the hubo model xml file. 
 * @param value The value that the upper limit should be set to
 */
void HuboMotor::setUpperLim(double value)
{
    //There is no upper limit error check. 
    upperLim = value;
}

/**
 * Set the lower limit for this joint, the limit is set at 
 * configuration time from the hubo model xml file. 
 * @param value The value that the upper limit should be set to
 */
void HuboMotor::setLowerLim(double value)
{
    //There is no lower limit error check. 
    lowerLim = value;
}