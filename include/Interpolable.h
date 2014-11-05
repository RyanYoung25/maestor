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

    // FeedBack Data
    double offset;          //The offset to add to each interpolation step (Dangerous and needs a lot of testing)

    FourthOrderParams startParams;
    FourthOrderParams currParams;
public:
    Interpolable();
    virtual ~Interpolable();

    void setFrequency(double frequency);
    bool setOffset(double offSet);
    double getOffset();

    virtual double interpolate();

};

#endif /* INTERPOLABLE_H_ */
