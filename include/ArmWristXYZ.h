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
    bool positionSet;
    bool isLeft;
    double SHOULDER_ROLL_LOWER;
    double SHOULDER_ROLL_UPPER;
    double SR_OFFSET;
    double SR_IK_OFFSET;
    double shoulder_yaw;

};

#endif /* ARMWRISTXYZ_H_ */
