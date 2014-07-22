/*
 * MetaJoint.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: solisknight
 */

#include "MetaJointController.h"

MetaJointController::MetaJointController(int numParameters, int numControlled) {
    NUM_PARAMETERS = numParameters;
    NUM_CONTROLLED = numControlled;
}

MetaJointController::~MetaJointController() {
    this->updated = true;
}

void MetaJointController::addParameter(RobotComponent* parameter){
    if (parameters.size() < NUM_PARAMETERS)
        parameters.push_back(parameter);
    else
        cout << "Attempt to add parameter " << parameter->getName() << " to already full meta joint controller." << endl;
}

void MetaJointController::addControlledJoint(RobotComponent* controlledJoint){
    if (controlledJoints.size() < NUM_CONTROLLED)
        controlledJoints.push_back(controlledJoint);
    else
        cout << "Attempt to add joint " << controlledJoint->getName() << " to already full meta joint controller." << endl;
}

int MetaJointController::getNumParameters(){
    return NUM_PARAMETERS;
}

int MetaJointController::getNumControlled(){
    return NUM_CONTROLLED;
}

bool MetaJointController::allSet(){
    double temp;
    for (int i = 0; i < parameters.size(); i++){
        parameters[i]->get(READY, temp);
        if (!temp)
            return false;
    }
    return true;
}

void MetaJointController::unsetAll(){
    for (int i = 0; i < parameters.size(); i++)
        parameters[i]->set(READY, false);
}

void MetaJointController::update(){
    updated = true;
}
void MetaJointController::goalsReached(){
    updated = false;
}
