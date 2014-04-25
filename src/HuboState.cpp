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
#include "HuboState.h"

HuboState::HuboState(){}

HuboState::~HuboState(){
    reset();
}

bool HuboState::setAlias(string name, string alias){
    if (!nameExists(name) || nameExists(alias)){
        cout << "Alias " << alias << "Already exists. returning false" << endl;
        return false;
    }

    index[alias] = index[name];

    return true;
}

bool HuboState::nameExists(string name){
    return index.count(name) == 1;
}

RobotComponent* HuboState::getComponent(string name){
    if (nameExists(name))
        return index[name];
    return NULL;
}

const HuboState::Components& HuboState::getComponents(){
    return components;
}

const HuboState::Motors& HuboState::getMotors(){
    return motors;
}

void HuboState::initHuboWithDefaults(string path, double frequency){
    xml_document doc;
    if (!doc.load_file(path.c_str())){
        cout << "No such file, " << path.c_str() << endl;
        return;
    }
    reset();
    xml_node robot = doc.child("robot");

    //Loop through each board
    cout << "Beginning to loop through each board" << endl;
    for (xml_node::iterator it = robot.begin(); it != robot.end(); it++) {
        xml_node node = *it;
        string type = node.attribute("type").as_string();       
        if (strcmp(type.c_str(), "HuboMotor") == 0){
            RobotComponent* component = HuboMotorFromXML(node, new HuboMotor(), frequency);
            if (component == NULL){
                cout << "Error instantiating " << type << " in initialization." << endl;
                continue;
            }
            if (!addComponentFromXML(node, component, true)){
                cout << "Error adding component " << component->getName() << endl;
                delete component;
                continue;
            }
            cout << "Adding component: " << component->getName() << endl;
            motors.push_back(static_cast<HuboMotor*>(component));

        } else if (strcmp(type.c_str(), "FTSensor") == 0) {
            RobotComponent* component = FTSensorFromXML(node, new FTSensorBoard());
            if (component == NULL){
                cout << "Error instantiating " << type << " in initialization." << endl;
                continue;
            }

            if (!addComponentFromXML(node, component, true)){
                cout << "Error adding component " << component->getName() << endl;
                delete component;
                continue;
            }

        } else if (strcmp(type.c_str(), "IMUSensor") == 0) {
            RobotComponent* component = IMUSensorFromXML(node, new IMUBoard());
            if (component == NULL){
                cout << "Error instantiating " << type << " in initialization." << endl;
                continue;
            }

            if (!addComponentFromXML(node, component, true)){
                cout << "Error adding component " << component->getName() << endl;
                delete component;
                continue;
            }

        } else if (strcmp(type.c_str(), "NeckRollPitch") == 0) {
            MetaJointController* controller = new NeckRollPitch();
            addMetaJointControllerFromXML(node, controller, type, frequency);

        } else if (strcmp(type.c_str(), "RightArmWristXYZ") == 0) {
            MetaJointController* controller = new ArmWristXYZ(false);
            addMetaJointControllerFromXML(node, controller, type, frequency);

        } else if (strcmp(type.c_str(), "LeftArmWristXYZ") == 0) {
            MetaJointController* controller = new ArmWristXYZ(true);
            addMetaJointControllerFromXML(node, controller, type, frequency);

        } else if (strcmp(type.c_str(), "LowerBodyLeg") == 0) {
            MetaJointController* controller = new LowerBodyLeg(false);
            addMetaJointControllerFromXML(node, controller, type, frequency);

        } else if (strcmp(type.c_str(), "GlobalLeg") == 0) {
            MetaJointController* controller = new LowerBodyLeg(true);
            addMetaJointControllerFromXML(node, controller, type, frequency);

        } else {
            cout << "Skipping unknown type " << type << endl;
            continue;
        }
    }
}

RobotComponent* HuboState::HuboMotorFromXML(xml_node node, HuboMotor* component, double frequency){
    if (!node.attribute("boardNum").empty())
        component->setBoardNum(node.attribute("boardNum").as_int());

    if (node.attribute("name").empty()){
        delete component;
        return NULL;
    }

    component->setFrequency(frequency);
    component->setName(node.attribute("name").as_string());

    return component;
}

RobotComponent* HuboState::FTSensorFromXML(xml_node node, FTSensorBoard* component){
    if (!node.attribute("boardNum").empty())
        component->setBoardNum(node.attribute("boardNum").as_int());

    if (node.attribute("name").empty()){
        delete component;
        return NULL;
    }

    component->setName(node.attribute("name").as_string());
    return component;
}

RobotComponent* HuboState::IMUSensorFromXML(xml_node node, IMUBoard* component){
    if (!node.attribute("boardNum").empty())
        component->setBoardNum(node.attribute("boardNum").as_int());

    if (node.attribute("name").empty()){
        delete component;
        return NULL;
    }

    component->setName(node.attribute("name").as_string());
    return component;
}

RobotComponent* HuboState::MetaJointFromXML(xml_node node, MetaJoint* component, double frequency){
    if (node.attribute("name").empty()){
        delete component;
        return NULL;
    }

    component->setName(node.attribute("name").as_string());

    if (!node.attribute("default").empty())
        component->setGoal(node.attribute("default").as_double());
    component->setFrequency(frequency);

    return component;
}

bool HuboState::addComponentFromXML(xml_node node, RobotComponent* component, bool back){
    if (nameExists(component->getName())){
        cout << "Skipping duplicate component with name " << component->getName() << endl;
        return false;
    }

    if (!node.attribute("vel").empty()){
        component->set(VELOCITY, node.attribute("vel").as_double());
    }

    if (back)
        components.push_back(component);
    else
        components.insert(components.begin(), component); //Note: This is extremely inefficient. Luckily, this is initialization.
    index[component->getName()] = component;

    xml_node aliases = node.child("aliases");
    for (xml_node::iterator values = aliases.begin(); values != aliases.end(); values++){
        string alias = (*values).child_value();
        setAlias(component->getName(), alias);
    }

    if(index[component->getName()] == NULL)
    {
        return false;
    }

    return true;
}

bool HuboState::addMetaJointControllerFromXML(xml_node node, MetaJointController* controller, string type, double frequency){
    vector<RobotComponent*> parameters;
    vector<xml_node> parameterNodes; // I guess I need this for aliases.... ah well.
    vector<RobotComponent*> controlled;

    for (xml_node::iterator it = node.begin(); it != node.end(); it++) {
        if (strcmp((*it).name(), "parameter") == 0){
            RobotComponent* component = MetaJointFromXML(*it, new MetaJoint(controller), frequency);
            if (component == NULL){
                cout << "Error instantiating parameter of " << type << " in initialization." << endl;
                return false;
            } else if (nameExists(component->getName())){
                cout << "Parameter name " << component->getName() << " of " << type << " already exists." << endl;
                delete component;
                return false;
            }
            parameters.push_back(component);
            parameterNodes.push_back(*it);

        } else if (strcmp((*it).name(), "controlled") == 0){
            if ((*it).attribute("name").empty()){
                cout << "Error instantiating controlled joint of " << type << " in initialization." << endl;
                return false;
            }

            string name = (*it).attribute("name").as_string();

            RobotComponent* component = getComponent(name);
            if (component == NULL){
                cout << "Could not find component " << name << " for type " << type << endl;
                return false;
            }

            controlled.push_back(component);
        }
    }

    bool errorFound = false;
    if (controller->getNumParameters() != parameters.size()) {
        cout << "Incorrect number of parameters passed to " << type << endl;
        errorFound = true;
    } else if (controller->getNumControlled() != controlled.size()){
        cout << "Incorrect number of controlled joints passed to " << type << endl;
        errorFound = true;
    } else if (parameters.size() != parameterNodes.size()){
        cout << "The unlikeliest error seems to have occurred.... abort mission? " << endl;
        errorFound = true;
    }

    if (errorFound){
        for (int i = 0; i < parameters.size(); i++)
            delete parameters[i];
        delete controller;
        return false;
    }

    for (int i = 0; i < parameters.size(); i++){
        controller->addParameter(parameters[i]);

        //Theoretically there's no way this can fail, so I don't check for errors here.
        addComponentFromXML(parameterNodes[i], parameters[i], false);
    }

    for (int i = 0; i < controlled.size(); i++)
        controller->addControlledJoint(controlled[i]);

    controllers.push_back(controller);
    return true;
}

void HuboState::reset(){
    cout << "Going to reset" << endl;
    if (components.size() != 0) {
        for (Components::iterator it = components.begin(); it != components.end(); it++)
            delete (*it);
    }
    if (controllers.size() != 0){
        for (vector<MetaJointController*>::iterator it = controllers.begin(); it != controllers.end(); it++)
            delete (*it);
    }

    cout << "Did we make it past the ifs?" << endl;
    components.clear();
    motors.clear();
    controllers.clear();
    index.clear();
}
