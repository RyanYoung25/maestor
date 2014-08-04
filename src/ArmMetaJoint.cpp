/**
 * The arm meta joint. This was to override the meta joint 
 * so that we don't need to interpolate the X Y and Z meta joints 
 * and instead just take them for what they are set to. 
 */

#include "ArmMetaJoint.h"

/**
 * Create the meta joint by giving it a meta joint controller
 */
ArmMetaJoint::ArmMetaJoint(MetaJointController* controller): MetaJoint(controller){
    this->controller = controller;
    currGoal = 0.0;
}

/**
 * Destructor
 */
ArmMetaJoint::~ArmMetaJoint() {}

/**
 * Get the property from this meta joint
 * @param  property The property to get
 * @param  value    A pointer to store the value of the property in 
 * @return          True on success
 */
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

/**
 * Set the value of a property to the value specified 
 * @param  property The property to set
 * @param  value    The value to set the property to 
 * @return          True on success
 */
bool ArmMetaJoint::set(PROPERTY property, double value){
    double currVel;
    double newVia;
    switch (property){
    case META_VALUE:
        position = value;
        break;
    case POSITION:
    case GOAL:
        controller->update();
        if(value == position){
            break;
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
 * Set the goal and interpolation set and old goal of the 
 * meta joint to the position.
 * @param pos What to set everything to
 */
void ArmMetaJoint::setGoal(double pos){
    currGoal = pos;
    interStep = pos;
    lastGoal = pos;
}
