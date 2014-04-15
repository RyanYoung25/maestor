/*
 * Interpolable.cpp
 *
 *  Created on: Mar 18, 2014
 *      Author: solisknight
 */

#include "Interpolable.h"

Interpolable::Interpolable() {
    currGoal = 0;
    interStep = 0;
    interVel = .3;

    lastGoal = 0;
    frequency = 0;
    currStepCount = 0;
    totalStepCount = 0;

    memset(&startParams, 0, sizeof(startParams));
    memset(&currParams, 0, sizeof(currParams));
}

Interpolable::~Interpolable() {
}


double Interpolable::interpolate(){
    double error = 0;
    double velocity = 0;
    double time = 0;
    if (totalStepCount != 0) {

        if (totalStepCount > currStepCount){
            /*
            error = currGoal - lastGoal;
            time = (double)currStepCount / (double)totalStepCount;

            interStep = lastGoal + interpolateSin(error, time);
            currStepCount++;
            return interStep;*/
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
    interStep += interpolateTrap(error, velocity);

    return interStep;
}

void Interpolable::setFrequency(double frequency){
    this->frequency = frequency;
}

/*
double HuboMotor::interpolate(){
    if (totalStepCount != 0)
        return interpolateSin();
    return interpolateTrap();
}*/

/*

double HuboMotor::interpolateTrap(){
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

double HuboMotor::interpolateSin(){
    if (totalStepCount == currStepCount){
        totalStepCount = 0;
        currStepCount = 0;
        return interpolateTrap();
    }
    //double error = currGoal - lastGoal;
    //double time = M_PI / totalStepCount * currStepCount;

    interStep = (lastGoal) + ( (currGoal - lastGoal) * .5 * (1.0 - cos( M_PI / totalStepCount * currStepCount ) ) );
    currStepCount++;
    return interStep;
}
*/
