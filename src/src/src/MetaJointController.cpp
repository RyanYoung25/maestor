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

/**
 * The super class for all metajoint controllers
 */

#include "MetaJointController.h"

/**
 * Create a meta joint controller
 */
MetaJointController::MetaJointController(int numParameters, int numControlled) {
    NUM_PARAMETERS = numParameters;
    NUM_CONTROLLED = numControlled;
}

/**
 * Destroy a metajoint controller
 */
MetaJointController::~MetaJointController() {
    this->updated = true;
}

/**
 * Add a parameter to the meta joint controller
 * @param parameter The robot component parameter to add
 */
void MetaJointController::addParameter(RobotComponent* parameter){
    if (parameters.size() < NUM_PARAMETERS)
        parameters.push_back(parameter);
    else
        cout << "Attempt to add parameter " << parameter->getName() << " to already full meta joint controller." << endl;
}

/**
 * Add a controlled joint to the controller
 * @param controlledJoint The joint to add that will be controlled
 */
void MetaJointController::addControlledJoint(RobotComponent* controlledJoint){
    if (controlledJoints.size() < NUM_CONTROLLED)
        controlledJoints.push_back(controlledJoint);
    else
        cout << "Attempt to add joint " << controlledJoint->getName() << " to already full meta joint controller." << endl;
}

/**
 * Return the number of parameters
 * @return The number of parameters
 */
int MetaJointController::getNumParameters(){
    return NUM_PARAMETERS;
}

/**
 * Return the number of controlled joints
 * @return The number of controlled joints
 */
int MetaJointController::getNumControlled(){
    return NUM_CONTROLLED;
}

/**
 * Check to see if all of the parameter joints are ready
 * @return True if they are all ready. false otherwise
 */
bool MetaJointController::allSet(){
    double temp;
    for (int i = 0; i < parameters.size(); i++){
        parameters[i]->get(READY, temp);
        if (!temp)
            return false;
    }
    return true;
}

/**
 * Unset all of the parameters, Make ready false
 */
void MetaJointController::unsetAll(){
    for (int i = 0; i < parameters.size(); i++)
        parameters[i]->set(READY, false);
}

/**
 * Signal that the controller was updated
 */
void MetaJointController::update(){
    updated = true;
}

/**
 * Signal that the goals were reached and therefore 
 * the controller needs to be updated again to have control
 */
void MetaJointController::goalsReached(){
    updated = false;
}
