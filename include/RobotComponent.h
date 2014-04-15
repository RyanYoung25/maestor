/*
 * RobotComponent.h
 *
 *  Created on: Feb 25, 2014
 *      Author: solisknight
 */

#ifndef ROBOTCOMPONENT_H_
#define ROBOTCOMPONENT_H_

#include <string>
#include "Names.h"
#include "StateChannel.h"

using std::string;

class RobotComponent {
public:
    RobotComponent();
    virtual ~RobotComponent();

    void setName(string name);
    string& getName();

    virtual bool get(PROPERTY property, double& value)=0;
    virtual bool set(PROPERTY property, double value)=0;

protected:
    StateChannel *stateChannel;

    string name;
};

#endif /* ROBOTCOMPONENT_H_ */
