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
 * LowerBodyLeg.h
 *
 *  Created on: Mar 5, 2014
 *      Author: solisknight
 */

#ifndef LOWERBODYLEG_H_
#define LOWERBODYLEG_H_

//#include <math.h>
#include <cmath>

#include "MetaJointController.h"

class LowerBodyLeg : public MetaJointController {
    static const int NUM_PARAMETERS;
    static const int NUM_CONTROLLED;

    //Expected Parameter Indices
    static const int FOOT_X;
    static const int FOOT_Y;
    static const int FOOT_Z;
    static const int FOOT_YAW;
    static const int FOOT_ROLL;
    static const int FOOT_PITCH;

    //Expected Controlled Indices
    static const int HIP_YAW;
    static const int HIP_ROLL;
    static const int HIP_PITCH;
    static const int KNEE_PITCH;
    static const int ANKLE_PITCH;
    static const int ANKLE_ROLL;

    //Constants which should be parameterized
    static const double LENGTH_THIGH;
    static const double LENGTH_CALF;
    static const double HIP_ROLL_UPPER;
    static const double HIP_ROLL_LOWER;
    static const double HIP_PITCH_UPPER;
    static const double HIP_PITCH_LOWER;
    static const double KNEE_PITCH_UPPER;
    static const double KNEE_PITCH_LOWER;
    static const double ANKLE_PITCH_UPPER;
    static const double ANKLE_PITCH_LOWER;
    static const double ANKLE_ROLL_UPPER;
    static const double ANKLE_ROLL_LOWER;


public:
    LowerBodyLeg(bool global);
    virtual ~LowerBodyLeg();

    void setInverse();
    void getForward();

    vector<double> getHipRollXYZ();
    void setHipRollXYZ(double foot_x, double foot_y, double foot_z, double foot_yaw);

private:
    void checkGoalsReached();
    bool globalFlag;
};

#endif /* LOWERBODYLEG_H_ */
