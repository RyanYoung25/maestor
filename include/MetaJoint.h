/*
 * MetaJoint.h
 *
 *  Created on: Mar 4, 2014
 *      Author: solisknight
 */

#ifndef METAJOINT_H_
#define METAJOINT_H_

#include "RobotComponent.h"
#include "Interpolable.h"
#include "MetaJointController.h"
    class MetaJointController;

class MetaJoint : public RobotComponent, public Interpolable {
private:
    MetaJointController* controller;
    double position;
    bool ready;

public:
    MetaJoint(MetaJointController* controller);
    virtual ~MetaJoint();

    bool get(PROPERTY property, double &value);
    bool set(PROPERTY property, double value);

    void setGoal(double pos);

};

#endif /* METAJOINT_H_ */
