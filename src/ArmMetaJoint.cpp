/*
 * ArmMetaJoint.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: solisknight
 */

#include "ArmMetaJoint.h"

ArmMetaJoint::ArmMetaJoint(MetaJointController* controller): MetaJoint(controller){
    cout << "Arm Meta Joint was Made" << endl;
    this->controller = controller;
    currGoal = 0.0;
}

ArmMetaJoint::~ArmMetaJoint() {}

bool ArmMetaJoint::get(PROPERTY property, double &value){
    switch (property){
    case ENABLED:
        value = true;
        break;
    case GOAL:
        value = currGoal;
        break;
    case INTERPOLATION_STEP:
        if (!ready){
            ready = true;
            controller->setInverse();
        } else
            value = currGoal;
        break;
    case POSITION:
        controller->getForward();
        value = position;
        break;
    case READY:
        value = ready;
        break;
    default:
        return false;
    }
    return true;
}

bool ArmMetaJoint::set(PROPERTY property, double value){
    double currVel;
    double newVia;
    switch (property){
    case META_VALUE:
        position = value;
        break;
    case POSITION:
    case GOAL:
        currGoal = value;
        break;
    case VELOCITY:
        interVel = value;
        break;
    case READY:
        ready = (bool)value;
        break;
    default:
        return false;
    }
    return true;
}

void ArmMetaJoint::setGoal(double pos){
    currGoal = pos;
    interStep = pos;
    lastGoal = pos;
}
