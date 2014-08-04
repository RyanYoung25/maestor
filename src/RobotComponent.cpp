/**
 * A super class for all robot components
 */

#include "RobotComponent.h"

/**
 * The constructor for a robot component
 */
RobotComponent::RobotComponent() {
    stateChannel = StateChannel::instance();
}

/**
 * Destructor
 */
RobotComponent::~RobotComponent() {}

/**
 * Set the name of a robot component. Because all components
 * have names. 
 * @param name The name to set
 */
void RobotComponent::setName(string name){
    this->name = name;
}

/**
 * Get the name of the robot component
 * @return The name of the component
 */
string& RobotComponent::getName(){
    return name;
}

