/**
 * A metajoint object. This is extended by ArmMetaJoint but for all 
 * the other metajoints in MAESTOR they are objects of this class
 */

#include "MetaJoint.h"

/**
 * Create a metajoint and tell it it's controller
 */
MetaJoint::MetaJoint(MetaJointController* controller){
    this->controller = controller;
    this->position = 0;
    this->ready = false;
}

/**
 * Destructor
 */
MetaJoint::~MetaJoint() {}

/**
 * Get the property of this metajoint. Store it in the pointer that is passed in. 
 * 
 * @param  property The property that you want to ask about
 * @param  value    A pointer to store the result in
 * @return          True on success
 */
bool MetaJoint::get(PROPERTY property, double &value){
    switch (property){
    case ENABLED:
        value = true;
        break;
    case GOAL:
        value = this->currGoal;
        break;
    case INTERPOLATION_STEP:
        if (!ready){
            ready = true;
            controller->setInverse();
        } else{
            value = interpolate();
        }
        break;
    case POSITION:
        controller->getForward();
        value = position;
        break;
    case READY:
        value = ready;
        break;
    case VELOCITY:
        value = interVel;
        break;
    default:
        return false;
    }
    return true;
}

/**
 * Set the property of this metajoint to the value passed in
 * 
 * @param  property The property to set
 * @param  value    The value to set it to
 * @return          True on success
 */
bool MetaJoint::set(PROPERTY property, double value){
    double currVel;
    double newVia;
    switch (property){
    case META_VALUE:
        position = value;
        break;
    case POSITION:
    case GOAL:
        controller->update();
        if (value == interStep){
            break;
        }
        currVel = (currStepCount != 0 && currParams.valid) ?
                (interpolateFourthOrder(currParams, currStepCount) - interpolateFourthOrder(currParams, currStepCount - 1)) : 0;

        if (!startParams.valid){

            currStepCount = 0;
            totalStepCount = totalTime(interStep, value, currVel, interVel) * frequency;

            if (totalStepCount > 0) {
                startParams = initFourthOrder(interStep, currVel, (value + interStep)/2, (double)totalStepCount / 2, value, totalStepCount);
                currParams = startParams;
            }

            lastGoal = interStep;
        } else if (currStepCount != currParams.tv) {

            newVia = (startParams.ths + interpolateFourthOrder(currParams, currStepCount));
            totalStepCount = currStepCount + (totalTime(newVia, value, currVel, interVel) * frequency);
            currParams = initFourthOrder( startParams.ths, currVel, newVia, currStepCount, value, totalStepCount );
        }
        currGoal = value;
        break;
    case SPEED:
    case VELOCITY:
        if (value > 0)
            interVel = value;
        break;
    case READY:
        ready = (bool)value;
        break;
    case INTERPOLATION_STEP:
        currParams.valid = false;
        startParams.valid = false;
        totalStepCount = 0;
        currStepCount = 0;
        break;
    default:
        return false;
    }
    return true;
}

/**
 * Set the goal and everything to that position
 * @param pos The position to set it all to
 */
void MetaJoint::setGoal(double pos){
    currGoal = pos;
    interStep = pos;
    lastGoal = pos;
}
