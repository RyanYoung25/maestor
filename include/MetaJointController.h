/*
 * MetaJoint.h
 *
 *  Created on: Mar 4, 2014
 *      Author: solisknight
 */

#ifndef METAJOINTCONTROLLER_H_
#define METAJOINTCONTROLLER_H_

#include <vector>
#include <iostream>
#include "RobotComponent.h"
#include "MetaJoint.h"
    class MetaJoint;

using std::vector;
using std::cout;
using std::endl;

class MetaJointController {
protected:
    vector<RobotComponent*> parameters;
    vector<RobotComponent*> controlledJoints;

private:
    int NUM_PARAMETERS;
    int NUM_CONTROLLED;

public:
    MetaJointController(int numParameters, int numControlled);
    virtual ~MetaJointController();

    virtual void setInverse()=0;
    virtual void getForward()=0;

    void addParameter(RobotComponent* parameter);
    void addControlledJoint(RobotComponent* controlledJoint);

    int getNumParameters();
    int getNumControlled();

    bool allSet();
    void unsetAll();
};

#endif /* METAJOINT_H_ */
