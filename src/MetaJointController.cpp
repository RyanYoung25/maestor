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
