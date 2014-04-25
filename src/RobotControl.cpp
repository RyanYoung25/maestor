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
#include "RobotControl.h"

RobotControl::RobotControl(){

	RUN_TYPE = HARDWARE;

    commandChannel = CommandChannel::instance();
    referenceChannel = ReferenceChannel::instance();
    stateChannel = StateChannel::instance();
    if(RUN_TYPE == SIMULATION){
        simChannels = SimChannels::instance();
    }

    this->written = 0;
    this->printNow = false;
    this->enableControl = false;
    this->delay = 0;
    this->state = NULL;
    this->interpolation = true;    //Interpret all commands as a final destination with given velocity.
    this->override = true;        //Force homing before allowing enabling. (currently disabled)

    commands["Enable"] = ENABLE;
    commands["EnableAll"] = ENABLEALL;
    commands["Disable"] = DISABLE;
    commands["DisableAll"] = DISABLEALL;
    commands["ResetJoint"] = RESET;
    commands["ResetAll"] = RESETALL;
    commands["Home"] = HOME;
    commands["HomeAll"] = HOMEALL;
    commands["InitializeSensors"] = INITSENSORS;
    commands["Update"] = UPDATE;
    commands["Zero"] = ZERO;
    commands["ZeroAll"] = ZEROALL;

    ostringstream logfile;
    logfile << LOG_PATH << "RobotControl.log";
    tempOutput.open(logfile.str().c_str());
    power = new PowerControlBoard();

    frames = 0;
    trajStarted = false;
}
  
RobotControl::~RobotControl(){
    delete state;
    delete power;
}

void RobotControl::updateHook(){

    if (state == NULL)
        return;

    if(RUN_TYPE == SIMULATION){
        simChannels->load();
    }
    referenceChannel->load();
    stateChannel->load();


    trajStarted = trajectories.hasRunning();

    Boards boards = state->getBoards();
    Motors motors = state->getMotorMap();
    Trajectory* traj = NULL;
    HuboMotor* motor = NULL;


    if (!boards.empty()) {
        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            traj = trajectories.inRunning(motor->getName());

            if (motor->isEnabled()){
                double pos = 0;

                if (motor->getMode() == HUBO_REF_MODE_COMPLIANT){
                    // Compliance
                    motor->setInterStep(motor->getPosition());
                    motor->setGoalPosition(motor->getPosition());
                    pos = motor->getPosition();

                } else if (trajStarted && traj){
                    // Trajectory Playback
                    pos = motor->getGoalPosition();
                    motor->setMode(HUBO_REF_MODE_REF);
                    if (!traj->nextPosition(motor->getName(), pos) && !traj->hasNext()){
                        cout << "Reading of trajectory positions has terminated." << endl << "> ";
                        cout.flush(); // Fixing flushing issue with the deployer, maybe....
                        trajectories.stopTrajectory(traj);
                        trajStarted = trajectories.hasRunning();
                    }
                    motor->setGoalPosition(pos);
                    motor->setInterStep(pos);

                } else if (interpolation){
                    if (motor->requiresMotion())
                        pos = motor->interpolate();
                    else
                        pos = motor->getGoalPosition();
                    power->addMotionPower(motor->getName(), (1/200));  //TODO: Get the period
                    motor->setMode(HUBO_REF_MODE_REF_FILTER);
                } else {
                    pos = motor->getGoalPosition();
                }
                referenceChannel->setReference(motor->getName(), pos, motor->getMode());
                string key(WRITE_KEY);
                traj = trajectories.get(key);
                if (trajStarted && traj){
                    double currPos = get(motor->getName(), "position");

                    if (!traj->nextPosition(motor->getName(), currPos)){
                        cout << "Writing of trajectory positions has terminated." << endl << "> ";
                        cout.flush(); // Fixing flushing issue with the deployer, maybe....
                        trajectories.stopTrajectory(traj);
                        trajStarted = trajectories.hasRunning();
                    }
                }

                while (!trajectories.getCurrentTriggers().empty()){
                    startTrajectory(trajectories.getCurrentTriggers().front());
                    trajectories.getCurrentTriggers().pop();
                }
            }
        }
        if (trajStarted)
            trajectories.advanceFrame();
    }

    power->addMotionPower("IDLE", (1/200)); //TODO: get the period


    //Write out a message if we have one
    referenceChannel->update();

    usleep(delay);
}

void RobotControl::setSimType(){
    simChannels = SimChannels::instance();
    RUN_TYPE = SIMULATION;
}

bool RobotControl::loadTrajectory(string name, string path, bool read){
    return trajectories.loadTrajectory(name, path, read);
}

bool RobotControl::ignoreFrom(string name, string col){
    return trajectories.ignoreFrom(name, col);
}

bool RobotControl::ignoreAllFrom(string name){
    return trajectories.ignoreAllFrom(name);
}

bool RobotControl::unignoreFrom(string name, string col){
    return trajectories.unignoreFrom(name, col);
}

bool RobotControl::unignoreAllFrom(string name){
    return trajectories.unignoreAllFrom(name);
}

bool RobotControl::setTrigger(string name, int frame, string target){
    return trajectories.setTrigger(name, frame, target);
}

bool RobotControl::extendTrajectory(string name, string path){
    return trajectories.extendTrajectory(name, path);
}

void RobotControl::startTrajectory(string name){
    Trajectory* traj = NULL;

    traj = trajectories.get(name);
    if (traj == NULL){
        std::cout << "No trajectory with name " << name << " loaded." << endl;
        return;
    }

    Motors motors = state->getMotorMap();
    if (traj->read_only()) {

        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            HuboMotor* motor = it->second;

            if (traj->contains(motor->getName())){
                if (!motor->isEnabled()){
                    cout << "Cannot start trajectory: references disabled motor " << motor->getName() << endl;
                    return;
                }
                double pos = get(motor->getName(), "position");
                if (fabs(pos - traj->startPosition(motor->getName())) > .1){
                    cout << "Cannot start trajectory: motor " << motor->getName() << " should be at " << traj->startPosition(motor->getName()) << endl;
                    return;
                }
            }
        }
    } else {
        Header header;

        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            HuboMotor* motor = it->second;
            if (motor->isEnabled())
                header.push_back(motor->getName());

        }
        traj->setHeader(header);
    }

    trajectories.startTrajectory(name);
}

void RobotControl::stopTrajectory(string name){
    trajectories.stopTrajectory(name);
    trajStarted = trajectories.hasRunning();
}

string RobotControl::getDefaultInitPath(string path){
    ifstream is;
    is.open(path.c_str());
    string temp;
    if (is.is_open()){
        do {
            getline(is, temp, '\n');
        } while (temp.compare("Init File:") != 0);

        getline(is, temp, '\n');
        is.close();
    } else
        cout << "Error. Config file nonexistent. Aborting." << endl;

    return temp;
}

void RobotControl::initRobot(string path){
    this->state = new HuboState();
    if (strcmp(path.c_str(), "") == 0)
    {
        path = getDefaultInitPath(CONFIG_PATH);
    }
    //@TODO: Check for file existence before initializing.
    this->state->initHuboWithDefaults(path, 200);  //TODO: get the period

    if (this->state == NULL)
    {
        std::cout << "Error. Initializing robot failed. Robot state is null." << std::endl;
    }

}

void RobotControl::set(string name, string property, double value){
    Motors motors = state->getMotorMap();

    if (motors.count(name) == 0){
        cout << "Error. Motor with name " << name << " is not on record. Aborting." << endl;
        return;
    }
    HuboMotor* motor = motors[name];

    Properties properties = state->getPropertyMap();
    if (properties.count(property) == 0){
        cout << "Error. No property with name " << property << " registered. Aborting." << endl;
        return;
    }

    switch (properties[property]){
    case POSITION:
        if (printNow) cout << "Setting position of motor " << name << " to " << value << " ." << endl;
        //power->addMotionPower(name, motor->getGoalPosition(), value);
        motor->setGoalPosition(value);
        break;
    case VELOCITY:
        if (printNow) cout << "Setting velocity of motor " << name << " to " << value << " ." << endl;
        motor->setInterVelocity(value);
        if (!interpolation) {
            cout << "Warning. RobotControl is not currently handling interpolation. " << endl <<
                    "Automatically enabling interpolation for velocity control." << endl << "> ";
            cout.flush();
            setMode("Interpolation",true);
        }
        break;
    case MOTION_TYPE:
        switch ((int)value) {
        case HUBO_REF_MODE_REF:
            if (printNow) cout << "Setting mode to direct reference control for motor " << motor->getName() << endl;
            motor->setMode(HUBO_REF_MODE_REF);
            break;
        case HUBO_REF_MODE_REF_FILTER:
            if (printNow) cout << "Setting mode to filtered reference control for motor " << motor->getName() << endl;
            motor->setMode(HUBO_REF_MODE_REF_FILTER);
            break;
        case HUBO_REF_MODE_COMPLIANT:
            if (printNow) cout << "Setting mode to compliant control for motor " << motor->getName() << endl;
            motor->setMode(HUBO_REF_MODE_COMPLIANT);
            break;
        case HUBO_REF_MODE_ENC_FILTER:
            if (printNow) cout << "Setting mode to encoder filtered control for motor " << motor->getName() << endl;
            motor->setMode(HUBO_REF_MODE_ENC_FILTER);
            break;
        // case HUBO_REF_MODE_REFX:
        //     if (printNow) cout << "Setting mode to differential reference control for motor " << motor->getName() << endl;
        //     motor->setMode(HUBO_REF_MODE_REFX);
        //     break;
        default:
            cout << "Motion type " << value << " not recognized." << endl;
            return;
        }
        break;
    default:
        cout << "Motor with name " << name << " has no mutable property named " << property << " ." << endl;
        return;
    }
}

void RobotControl::setProperties(string names, string properties, string values){
    vector<string> namesList = splitFields(names);
    vector<string> propertiesList = splitFields(properties);
    vector<string> valuesList = splitFields(values);
    if (namesList.size() != propertiesList.size()
            || namesList.size() != valuesList.size()
            || propertiesList.size() != valuesList.size()){
        cout << "Error! Size of entered fields not consistent. Aborting." << endl;
        return;
    }

    for (int i = 0; i < namesList.size(); i++){
        istringstream data(valuesList[i]);
        double value = 0;
        data >> value;

        set(namesList[i], propertiesList[i], value);
    }
}

double RobotControl::get(string name, string property){
    Motors motors = state->getMotorMap();
    FTSensors ftSensors = state->getFTSensorMap();
    IMUSensors imuSensors = state->getIMUSensorMap();
    Properties properties = state->getPropertyMap();

    double result = 0;

    if (properties.count(property) == 0){
        cout << "Error. No property with name " << property << " registered. Aborting." << endl;
        return 0;
    }

    if (motors.count(name) == 1){

        if (!stateChannel->getMotorProperty(name, properties[property], result)){
            cout << "Error requesting property " << property << " from state channel." << endl;
            return 0;
        }
        return result;

    } else if (ftSensors.count(name) == 1){

        if (!stateChannel->getFTProperty(name, properties[property], result)){
            cout << "Error requesting property " << property << " from state channel." << endl;
            return 0;
        }
        return result;

    } else if (imuSensors.count(name) == 1){

        if (!stateChannel->getIMUProperty(name, properties[property], result)){
            std::cout << "Error requesting property " << property << " from state channel." << std::endl;
            return 0;
        }
        return result;

    } else if (name.compare("PWR") == 0){
        return power->getTotalPowerUsed();
    } else {
        cout << "Error. Readable Object with name " << name << " is not on record. Aborting." << endl;
        return 0;
    }
}

string RobotControl::getProperties(string name, string properties) {
    vector<string> propertyList = splitFields(properties);
    ostringstream values;
    for (int i = 0; i < propertyList.size(); ++i)
    {
        double tmp = get(name, propertyList[i]);
        values << tmp;
        if(i + 1 != propertyList.size())
        {
            values << ", ";
        }
    }
    return values.str();
}

void RobotControl::command(string name, string target){
    Motors motors = state->getMotorMap();

    HuboMotor* motor;

    //only for normalization
    double pos;
    string joint;

    if (commands.count(name) == 0){
        cout << "Error. No command with name " << name << " is defined for RobotControl. Aborting." << endl << "> ";
        cout.flush();
        return;
    }

    switch (commands[name]){
    case ENABLE:
        if (motors.count(target) == 0){
            cout << "Error. Motor with name " << target << " is not on record. Aborting." << endl << "> ";
            cout.flush();
            return;
        }

        if (!this->commandChannel->enable(target)){
            cout << "Enable command failed. Aborting." << endl;
            return;
        }

        motor = motors[target];
        motor->setGoalPosition(get(motor->getName(), "position"));

        motor->setEnabled(true);
        /*
        if (RUN_TYPE == HARDWARE && !motor->isHomed()){
            cout << "Warning! Motor " << target << " has not yet been homed. Skipping enabling of this motor." << endl;
            return;
        }
        if (RUN_TYPE == HARDWARE)
            motor->setGoalPosition(motor->getPosition());
            */

        break;
    case ENABLEALL:

        if (!this->commandChannel->enable("all")){
            cout << "Enable command failed. Aborting." << endl;
            return;
        }

        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;

            /*
            if (RUN_TYPE == HARDWARE && !motor->isHomed()){
                cout << "Warning! Motor " << motor->getName() << " has not yet been homed. Skipping enabling of this motor." << endl;
                continue;
            }
            if (RUN_TYPE == HARDWARE)
                motor->setGoalPosition(motor->getPosition());
            */
            motor->setGoalPosition(get(motor->getName(), "position"));
            motor->setEnabled(true);
        }

        break;
    case DISABLE:
        if (motors.count(target) == 0){
            cout << "Error. Motor with name " << target << " is not on record. Aborting.";
            return;
        }

        if (!this->commandChannel->disable(target)){
            cout << "Disable command failed. Aborting." << endl;
            return;
        }

        motor = motors[target];
        motor->setEnabled(false);
        break;
    case DISABLEALL:
        if (!this->commandChannel->disable("all")){
            cout << "Disable command failed. Aborting." << endl;
            return;
        }


        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            motor->setEnabled(false);
        }
        break;
    case RESET:
        if (motors.count(target) == 0){
            cout << "Error. Motor with name " << target << " is not on record. Aborting.";
            return;
        }

        if (!this->commandChannel->reset(target)){
            cout << "Reset command failed. Aborting." << endl;
            return;
        }
        break;
    case RESETALL:
        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            command("ResetJoint", motor->getName());
        }
        return;
    case HOME:
        if (motors.count(target) == 0){
            cout << "Error. Motor with name " << target << " is not on record. Aborting.";
            return;
        }

        if (!this->commandChannel->home(target)){
            cout << "Homing command failed. Aborting." << endl;
            return;
        }

        motor = motors[target];
        set(target, "position", 0);
        break;
    case HOMEALL:
        if (!this->commandChannel->home("all")){
            cout << "Homing command failed. Aborting." << endl;
            return;
        }

        for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            set(motor->getName(), "position", 0);
        }
        break;
        //TODO: Find a way to pause for a length of time here.
    case INITSENSORS:

        if (!this->commandChannel->initializeSensors()){
            cout << "Initialize Sensors command failed. Aborting." << endl;
            return;
        }
        break;
    case ZERO:
		if (motors.count(target) == 0){
			std::cout << "Error. Motor with name " << target << " is not on record. Aborting.";
			return;
		}
		set(target, "position", 0);
		break;
	case ZEROALL:
	    for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
            motor = it->second;
            set(motor->getName(), "position", 0);
        }
		break;
	case UPDATE:
        //updateState();
        break;
    }
}

void RobotControl::setMode(string mode, bool value){
    Motors motors = state->getMotorMap();
    HuboMotor* motor = NULL;

    if (mode.compare("Interpolation") == 0){
        if (printNow) cout << "Setting interpolation " << (value ? "on." : "off.") << endl;

        //If we are switching to interpolation, the internal step of each motor must be updated.
        if (value && !this->state->getBoards().empty()){

            for (Motors::iterator it = motors.begin(); it != motors.end(); it++){
                motor = it->second;
                if (motor->isEnabled()){
                    double pos = motor->getInterStep();
                    if (!stateChannel->getMotorProperty(motor->getName(), POSITION, pos))
                        continue;
                    motor->setInterStep(pos);

                }
            }
        }
        interpolation = value;
    } else {
        cout << "RobotControl does not have a mutable mode with name " << mode << "." << endl;
    }
}

bool RobotControl::setAlias(string name, string alias){
    Motors motors = state->getMotorMap();
    FTSensors ftSensors = state->getFTSensorMap();
    IMUSensors imuSensors = state->getIMUSensorMap();
    Properties properties = state->getPropertyMap();

    int entries = 0;

    entries += motors.count(alias);
    entries += ftSensors.count(alias);
    entries += imuSensors.count(alias);
    entries += properties.count(alias);
    entries += commands.count(alias);

    if (entries > 0){
        cout << "There already exists an entity named " << alias << " in RobotControl." << cout;
        return false;
    } else if (motors.count(name) == 1)
        motors[alias] = motors[name];
    else if (ftSensors.count(name) == 1)
        ftSensors[alias] = ftSensors[name];
    else if (imuSensors.count(name) == 1)
        imuSensors[alias] = imuSensors[name];
    else if (properties.count(name) == 1)
        properties[alias] = properties[name];
    else if (commands.count(name) == 1)
        commands[alias] = commands[name];

    return true;
}

vector<string> RobotControl::splitFields(string input){
    vector<string> output;
    int whitespaceType = 0;
    if (input.find(' ') != string::npos) whitespaceType += 1;
    if (input.find('\t') != string::npos) whitespaceType += 2;
    if (input.find('\n') != string::npos) whitespaceType += 4;

    char whitespace;
    switch (whitespaceType){
    case 1:
        whitespace = ' ';
        break;
    case 2:
        whitespace = '\t';
        break;
    case 4:
        whitespace = '\n';
        break;
    default:
        output.push_back(input);
        return output;
    }
    string field;
    int pos = 0;

    do {
        pos = input.find(whitespace);
        field = input.substr(0, pos);
        output.push_back(field);
        input = input.substr(pos + 1, input.length() - pos - 1);
    } while (input.find(whitespace) != string::npos);
    output.push_back(input);
    return output;
}

void RobotControl::setDelay(int us){
    this->delay = us;
}

bool RobotControl::requiresMotion(string name){
    Motors motors = state->getMotorMap();
    if (motors.count(name) == 0){
        cout << "Error. Motor with name " << name << " is not on record. Aborting." << endl;
        return false;
    }
    return motors[name]->requiresMotion();
}
