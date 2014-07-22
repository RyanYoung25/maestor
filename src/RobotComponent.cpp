/*
 * RobotComponent.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: solisknight
 */

#include "RobotComponent.h"

RobotComponent::RobotComponent() {
    stateChannel = StateChannel::instance();
}

RobotComponent::~RobotComponent() {}

void RobotComponent::setName(string name){
    this->name = name;
}

string& RobotComponent::getName(){
    return name;
}

