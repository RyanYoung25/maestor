/*
 * Interpolable.h
 *
 *  Created on: Mar 18, 2014
 *      Author: solisknight
 */

#ifndef INTERPOLABLE_H_
#define INTERPOLABLE_H_

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "Interpolation.h"


class Interpolable {
protected:
    //Output Data
    double currGoal;        //Goal position in radians
    double interStep;       //Current interpolated step in radians
    double interVel;        //Current interpolated velocity in rad/sec

    double lastGoal;        //Origin point for sinusoidal inteprolation
    double frequency;       //Interpolation Frequency
    int currStepCount;      //Step count of sinusoidal trajectory generation.
    int totalStepCount;     //Total step count of sinusoidal trajectory

    FourthOrderParams startParams;
    FourthOrderParams currParams;
public:
    Interpolable();
    virtual ~Interpolable();

    void setFrequency(double frequency);

    double interpolate();

};

#endif /* INTERPOLABLE_H_ */
