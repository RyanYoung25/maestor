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
