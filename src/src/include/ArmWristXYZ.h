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

#ifndef ARMWRISTXYZ_H_
#define ARMWRISTXYZ_H_

#include <math.h>

#include "MetaJointController.h"

class ArmWristXYZ : public MetaJointController {
    static const int NUM_PARAMETERS;
    static const int NUM_CONTROLLED;

    //Expected Parameter Indices
    static const int WRIST_X;
    static const int WRIST_Y;
    static const int WRIST_Z;
    
    //Expected Controlled Indices
    static const int SHOULDER_YAW;
    static const int SHOULDER_PITCH;
    static const int SHOULDER_ROLL;
    static const int ELBOW_PITCH;

    //Constants which should be parameterized
    static const double UPPER_ARM_Z;
    static const double UPPER_ARM_X;
    static const double LOWER_ARM_Z;
    static const double LOWER_ARM_X;
    static const double ARM_MIN_REACH;
    static const double RSP_OFFSET;
    static const double REP_OFFSET;
    static const double SHOULDER_PITCH_UPPER;
    static const double SHOULDER_PITCH_LOWER;
    static const double ELBOW_PITCH_UPPER;
    static const double ELBOW_PITCH_LOWER;

public:
    ArmWristXYZ(bool left);
    virtual ~ArmWristXYZ();


    void setInverse();
    void getForward();

private:
    void checkGoalsReached();
    bool positionSet;
    bool isLeft;
    bool jointsSet;
    double SHOULDER_ROLL_LOWER;
    double SHOULDER_ROLL_UPPER;
    double SR_OFFSET;
    double SR_IK_OFFSET;
    double shoulder_yaw;

};

#endif /* ARMWRISTXYZ_H_ */
