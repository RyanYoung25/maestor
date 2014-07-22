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
