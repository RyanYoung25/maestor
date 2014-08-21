/*
 * ArmMetaJoint.h
 *
 *  Created on: Mar 4, 2014
 *      
 */

#ifndef ARMMETAJOINT_H_
#define ARMMETAJOINT_H_

#include "RobotComponent.h"
#include "Interpolable.h"
#include "MetaJointController.h"
    class MetaJointController;

class ArmMetaJoint : public MetaJoint {
private:
    MetaJointController* controller;
    double position;
    bool ready;
    double currGoal;

public:
    ArmMetaJoint(MetaJointController* controller);
    virtual ~ArmMetaJoint();

    bool get(PROPERTY property, double &value);
    bool set(PROPERTY property, double value);

    void setGoal(double pos);

};

#endif /* ARMMETAJOINT_H_ */
