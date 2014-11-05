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

#include "Interpolable.h"

/**
 * Create an Interpolable object. This is mostly a class that objects 
 * that can interpolate inherit from. 
 */
Interpolable::Interpolable() {
    currGoal = 0;
    interStep = 0;
    interVel = .3;

    lastGoal = 0;
    frequency = 0;
    currStepCount = 0;
    totalStepCount = 0;

    offset = 0.0;

    memset(&startParams, 0, sizeof(startParams));
    memset(&currParams, 0, sizeof(currParams));
}

/**
 * Destructor
 */
Interpolable::~Interpolable() {
}

/**
 * Run one interpolation step. 
 * @return The next interpolated value
 */
double Interpolable::interpolate(){
    double error = 0;
    double velocity = 0;
    double time = 0;
    if (totalStepCount != 0) {

        if (totalStepCount > currStepCount){
            time = currStepCount;
            interStep = lastGoal + interpolateFourthOrder(currParams, time);
            startParams.valid = currParams.valid;
            currStepCount++;
            return interStep;
        }
        startParams.valid = false;
        currParams.valid = false;
        totalStepCount = 0;
        currStepCount = 0;

    }

    if (fabs(interStep - currGoal) < .001)
        return interStep;

    if (frequency == 0)
        return interStep;

    error = currGoal - interStep;
    velocity = interVel / frequency;
    interStep = interStep + offset + interpolateTrap(error, velocity);

    return interStep;
}

/**
 * Set the frequency
 * @param frequency The new frequency
 */
void Interpolable::setFrequency(double frequency){
    this->frequency = frequency;
}

/**
 * Set the interpolation offset
 * @param  offSet The offset to set
 * @return        True if it was successful 
 */
bool Interpolable::setOffset(double offSet){
    offset = offSet;
    return true;
}

/**
 * Get the offset
 * @return the offset
 */
double Interpolable::getOffset()
{
    return offset;
}
